// ============================================================================
// Monte Carlo Localization (MCL / Particle Filter)
// src/mcl.cpp
//
// DELETE THIS FILE (and include/mcl.h) if you want to remove MCL.
// No other files reference mcl.h unless you add those includes yourself.
// ============================================================================

#include "vex.h"         // Brain, wait, thread, distance, using namespace vex
#include "motor-control.h" // x_pos, y_pos, correct_angle, field_half_size, getInertialHeading
#include "utils.h"       // degToRad, radToDeg
#include "mcl.h"


// ============================================================================
// TUNING PARAMETERS
// ============================================================================

// Sensor noise - standard deviation of distance sensor readings (inches).
// Increase if MCL jumps around; decrease for tighter corrections.
static const float MCL_SENSOR_SIGMA = 2.5f;

// Motion noise - fraction of delta added as Gaussian noise.
// E.g. 0.05 = 5% of each inch of travel becomes noise.
static const float MCL_TRANS_NOISE = 0.05f;
static const float MCL_ROT_NOISE   = 0.05f;

// Minimum per-step noise floor (prevents particles from collapsing).
static const float MCL_TRANS_NOISE_FLOOR = 0.03f; // inches per update
static const float MCL_ROT_NOISE_FLOOR   = 0.01f; // radians per update

// Initial spread when initializing around a known pose.
static const float MCL_INIT_SPREAD_IN  = 3.0f;   // inches
static const float MCL_INIT_SPREAD_RAD = 0.052f;  // ~3 degrees

// MCL update period (milliseconds).
static const int MCL_UPDATE_MS = 50;

// Effective particle count threshold for resampling (fraction of N).
// Resample when N_eff < MCL_RESAMPLE_THRESH * MCL_NUM_PARTICLES.
static const float MCL_RESAMPLE_THRESH = 0.5f;


// ============================================================================
// PARTICLE STATE
// ============================================================================

struct MCLParticle {
    float x, y;    // position in inches
    float theta;   // heading in radians (0 = north, CW positive)
    float weight;
};

static MCLParticle mcl_particles[MCL_NUM_PARTICLES];

// Published pose estimate - written only by the MCL thread.
static volatile float mcl_est_x     = 0.0f;
static volatile float mcl_est_y     = 0.0f;
static volatile float mcl_est_theta = 0.0f; // radians

static bool mcl_running = false;


// ============================================================================
// MATH HELPERS
// ============================================================================

// Uniform random in [0, 1).
static float mcl_randf() {
    return (float)rand() / ((float)RAND_MAX + 1.0f);
}

// Gaussian sample with given standard deviation (Box-Muller transform).
static float mcl_randGauss(float stddev) {
    float u1 = mcl_randf() + 1e-9f; // avoid log(0)
    float u2 = mcl_randf();
    return stddev * sqrtf(-2.0f * logf(u1)) * cosf(2.0f * (float)M_PI * u2);
}

// Wrap angle to (-pi, pi].
static float mcl_wrapAngle(float a) {
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a <= -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

// Distance from point (sx, sy) along unit direction (dx, dy) to the nearest
// field wall (walls at +/- field_half_size on both axes). Returns inches.
static float mcl_rayToWall(float sx, float sy, float dx, float dy) {
    float half = (float)field_half_size;
    float t    = 1e9f;

    if (dx > 1e-6f) {
        float tx = (half - sx) / dx;
        if (tx > 0.0f && tx < t) t = tx;
    } else if (dx < -1e-6f) {
        float tx = (-half - sx) / dx;
        if (tx > 0.0f && tx < t) t = tx;
    }

    if (dy > 1e-6f) {
        float ty = (half - sy) / dy;
        if (ty > 0.0f && ty < t) t = ty;
    } else if (dy < -1e-6f) {
        float ty = (-half - sy) / dy;
        if (ty > 0.0f && ty < t) t = ty;
    }

    return t;
}

// Gaussian probability: P(measured | expected, sigma).
static float mcl_likelihood(float measured, float expected, float sigma) {
    float d = measured - expected;
    return expf(-0.5f * d * d / (sigma * sigma));
}


// ============================================================================
// MOTION UPDATE
// ============================================================================

// Propagate all particles by (dx, dy, dtheta) with added Gaussian noise.
// dx/dy are world-frame odometry deltas (inches); dtheta is radians.
static void mcl_motionUpdate(float dx, float dy, float dtheta) {
    float trans = sqrtf(dx * dx + dy * dy);
    float trans_sigma  = MCL_TRANS_NOISE * trans  + MCL_TRANS_NOISE_FLOOR;
    float rot_sigma    = MCL_ROT_NOISE   * fabsf(dtheta) + MCL_ROT_NOISE_FLOOR;
    float field_lim    = (float)field_half_size - 2.0f;

    for (int i = 0; i < MCL_NUM_PARTICLES; i++) {
        mcl_particles[i].x     += dx     + mcl_randGauss(trans_sigma);
        mcl_particles[i].y     += dy     + mcl_randGauss(trans_sigma);
        mcl_particles[i].theta  = mcl_wrapAngle(
            mcl_particles[i].theta + dtheta + mcl_randGauss(rot_sigma));

        // Clamp inside field boundaries (2" margin from walls).
        if (mcl_particles[i].x < -field_lim) mcl_particles[i].x = -field_lim;
        if (mcl_particles[i].x >  field_lim) mcl_particles[i].x =  field_lim;
        if (mcl_particles[i].y < -field_lim) mcl_particles[i].y = -field_lim;
        if (mcl_particles[i].y >  field_lim) mcl_particles[i].y =  field_lim;
    }
}


// ============================================================================
// SENSOR UPDATE
// ============================================================================

// Update particle weights using all four distance sensors.
//
// Coordinate convention (heading theta, 0 = north, CW positive):
//   Forward direction  : (sin(t),  cos(t))
//   Right direction    : (cos(t), -sin(t))
//   Backward direction : (-sin(t), -cos(t))
//   Left direction     : (-cos(t), sin(t))
//
// Sensor offsets (from robot-config):
//   front_sensor_offset: sensor face is this far forward of robot center
//   back_sensor_offset:  sensor face is this far behind robot center
//   left_sensor_offset:  sensor face is this far to the left of robot center
//   right_sensor_offset: sensor face is this far to the right of robot center
static void mcl_sensorUpdate() {
    // Read sensors - only use readings in valid range (20 - 2500 mm = ~0.8 - 98 in).
    float dist_f = -1.0f, dist_b = -1.0f, dist_l = -1.0f, dist_r = -1.0f;

    double raw;
    raw = front_sensor.objectDistance(mm);
    if (raw > 20.0 && raw < 2500.0) dist_f = (float)(raw / 25.4);

    raw = back_sensor.objectDistance(mm);
    if (raw > 20.0 && raw < 2500.0) dist_b = (float)(raw / 25.4);

    raw = left_sensor.objectDistance(mm);
    if (raw > 20.0 && raw < 2500.0) dist_l = (float)(raw / 25.4);

    raw = right_sensor.objectDistance(mm);
    if (raw > 20.0 && raw < 2500.0) dist_r = (float)(raw / 25.4);

    int valid = (dist_f > 0.0f) + (dist_b > 0.0f) + (dist_l > 0.0f) + (dist_r > 0.0f);
    if (valid == 0) return; // No usable sensor data - skip this update.

    float fo = (float)front_sensor_offset;
    float bo = (float)back_sensor_offset;
    float lo = (float)left_sensor_offset;
    float ro = (float)right_sensor_offset;

    float total_weight = 0.0f;

    for (int i = 0; i < MCL_NUM_PARTICLES; i++) {
        float px = mcl_particles[i].x;
        float py = mcl_particles[i].y;
        float t  = mcl_particles[i].theta;
        float st = sinf(t);
        float ct = cosf(t);
        float w  = 1.0f;

        if (dist_f > 0.0f) {
            // Front sensor: offset forward, pointing forward
            float sx = px + st * fo;
            float sy = py + ct * fo;
            float exp_d = mcl_rayToWall(sx, sy, st, ct);
            w *= mcl_likelihood(dist_f, exp_d, MCL_SENSOR_SIGMA);
        }

        if (dist_b > 0.0f) {
            // Back sensor: offset backward, pointing backward
            float sx = px - st * bo;
            float sy = py - ct * bo;
            float exp_d = mcl_rayToWall(sx, sy, -st, -ct);
            w *= mcl_likelihood(dist_b, exp_d, MCL_SENSOR_SIGMA);
        }

        if (dist_l > 0.0f) {
            // Left sensor: offset to the left, pointing left
            float sx = px - ct * lo;
            float sy = py + st * lo;
            float exp_d = mcl_rayToWall(sx, sy, -ct, st);
            w *= mcl_likelihood(dist_l, exp_d, MCL_SENSOR_SIGMA);
        }

        if (dist_r > 0.0f) {
            // Right sensor: offset to the right, pointing right
            float sx = px + ct * ro;
            float sy = py - st * ro;
            float exp_d = mcl_rayToWall(sx, sy, ct, -st);
            w *= mcl_likelihood(dist_r, exp_d, MCL_SENSOR_SIGMA);
        }

        mcl_particles[i].weight *= w;
        total_weight += mcl_particles[i].weight;
    }

    if (total_weight > 1e-15f) {
        float inv = 1.0f / total_weight;
        for (int i = 0; i < MCL_NUM_PARTICLES; i++) {
            mcl_particles[i].weight *= inv;
        }
    } else {
        // Particle deprivation: all particles died - reset to uniform.
        float uniform = 1.0f / MCL_NUM_PARTICLES;
        for (int i = 0; i < MCL_NUM_PARTICLES; i++) {
            mcl_particles[i].weight = uniform;
        }
    }
}


// ============================================================================
// SYSTEMATIC RESAMPLING
// ============================================================================

static void mcl_resample() {
    // Compute effective particle count: N_eff = 1 / sum(w^2).
    float sum_sq = 0.0f;
    for (int i = 0; i < MCL_NUM_PARTICLES; i++) {
        sum_sq += mcl_particles[i].weight * mcl_particles[i].weight;
    }
    float n_eff = (sum_sq > 1e-20f) ? (1.0f / sum_sq) : (float)MCL_NUM_PARTICLES;

    // Only resample if particle diversity has dropped below threshold.
    if (n_eff > MCL_RESAMPLE_THRESH * (float)MCL_NUM_PARTICLES) return;

    // Systematic resampling using a single random offset.
    static MCLParticle temp[MCL_NUM_PARTICLES];

    float cumsum[MCL_NUM_PARTICLES];
    cumsum[0] = mcl_particles[0].weight;
    for (int i = 1; i < MCL_NUM_PARTICLES; i++) {
        cumsum[i] = cumsum[i - 1] + mcl_particles[i].weight;
    }

    float step  = 1.0f / (float)MCL_NUM_PARTICLES;
    float start = mcl_randf() * step;
    int   j     = 0;

    for (int i = 0; i < MCL_NUM_PARTICLES; i++) {
        float target = start + (float)i * step;
        while (j < MCL_NUM_PARTICLES - 1 && cumsum[j] < target) j++;
        temp[i]        = mcl_particles[j];
        temp[i].weight = step;
    }

    for (int i = 0; i < MCL_NUM_PARTICLES; i++) {
        mcl_particles[i] = temp[i];
    }
}


// ============================================================================
// POSE ESTIMATION
// ============================================================================

static void mcl_estimatePose() {
    float wx      = 0.0f;
    float wy      = 0.0f;
    float sin_sum = 0.0f;
    float cos_sum = 0.0f;
    float total   = 0.0f;

    for (int i = 0; i < MCL_NUM_PARTICLES; i++) {
        float w  = mcl_particles[i].weight;
        wx      += w * mcl_particles[i].x;
        wy      += w * mcl_particles[i].y;
        sin_sum += w * sinf(mcl_particles[i].theta);
        cos_sum += w * cosf(mcl_particles[i].theta);
        total   += w;
    }

    if (total > 1e-15f) {
        float inv  = 1.0f / total;
        mcl_est_x     = wx * inv;
        mcl_est_y     = wy * inv;
        // Circular mean for angle avoids wrap-around averaging errors.
        mcl_est_theta = atan2f(sin_sum * inv, cos_sum * inv);
    }
}


// ============================================================================
// BACKGROUND THREAD
// ============================================================================

static void mcl_task_fn() {
    float prev_x     = (float)x_pos;
    float prev_y     = (float)y_pos;
    float prev_theta = (float)degToRad(getInertialHeading());

    while (true) {
        float curr_x     = (float)x_pos;
        float curr_y     = (float)y_pos;
        float curr_theta = (float)degToRad(getInertialHeading());

        float dx     = curr_x - prev_x;
        float dy     = curr_y - prev_y;
        float dtheta = mcl_wrapAngle(curr_theta - prev_theta);

        mcl_motionUpdate(dx, dy, dtheta);
        mcl_sensorUpdate();
        mcl_resample();
        mcl_estimatePose();

        prev_x     = curr_x;
        prev_y     = curr_y;
        prev_theta = curr_theta;

        wait(MCL_UPDATE_MS, msec);
    }
}


// ============================================================================
// PUBLIC API
// ============================================================================

void mclInit() {
    srand((unsigned int)Brain.timer(msec));

    float cx = (float)x_pos;
    float cy = (float)y_pos;
    float ct = (float)degToRad(getInertialHeading());
    float w  = 1.0f / MCL_NUM_PARTICLES;

    for (int i = 0; i < MCL_NUM_PARTICLES; i++) {
        mcl_particles[i].x      = cx + mcl_randGauss(MCL_INIT_SPREAD_IN);
        mcl_particles[i].y      = cy + mcl_randGauss(MCL_INIT_SPREAD_IN);
        mcl_particles[i].theta  = mcl_wrapAngle(ct + mcl_randGauss(MCL_INIT_SPREAD_RAD));
        mcl_particles[i].weight = w;
    }

    mcl_est_x     = cx;
    mcl_est_y     = cy;
    mcl_est_theta = ct;

    if (!mcl_running) {
        mcl_running = true;
        thread mcl_thread(mcl_task_fn);
    }
}

void mclInitUniform() {
    srand((unsigned int)Brain.timer(msec));

    float half = (float)field_half_size;
    float w    = 1.0f / MCL_NUM_PARTICLES;

    for (int i = 0; i < MCL_NUM_PARTICLES; i++) {
        mcl_particles[i].x      = (mcl_randf() * 2.0f - 1.0f) * half;
        mcl_particles[i].y      = (mcl_randf() * 2.0f - 1.0f) * half;
        mcl_particles[i].theta  = (mcl_randf() * 2.0f - 1.0f) * (float)M_PI;
        mcl_particles[i].weight = w;
    }

    if (!mcl_running) {
        mcl_running = true;
        thread mcl_thread(mcl_task_fn);
    }
}

void mclReset() {
    float cx = (float)x_pos;
    float cy = (float)y_pos;
    float ct = (float)degToRad(getInertialHeading());
    float w  = 1.0f / MCL_NUM_PARTICLES;

    for (int i = 0; i < MCL_NUM_PARTICLES; i++) {
        mcl_particles[i].x      = cx + mcl_randGauss(MCL_INIT_SPREAD_IN);
        mcl_particles[i].y      = cy + mcl_randGauss(MCL_INIT_SPREAD_IN);
        mcl_particles[i].theta  = mcl_wrapAngle(ct + mcl_randGauss(MCL_INIT_SPREAD_RAD));
        mcl_particles[i].weight = w;
    }

    mcl_est_x     = cx;
    mcl_est_y     = cy;
    mcl_est_theta = ct;
}

double mclGetX()     { return (double)mcl_est_x; }
double mclGetY()     { return (double)mcl_est_y; }
double mclGetTheta() { return radToDeg((double)mcl_est_theta); }

void mclApply() {
    x_pos         = (double)mcl_est_x;
    y_pos         = (double)mcl_est_y;
    correct_angle = radToDeg((double)mcl_est_theta);
}
