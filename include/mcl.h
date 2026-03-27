#pragma once

// ============================================================================
// Monte Carlo Localization (MCL / Particle Filter)
// ============================================================================
//
// Uses the 4 distance sensors (front, back, left, right) to correct odometry
// drift by weighing particles against expected wall distances.
//
// Coordinate system: same as odometry - x=east, y=north, heading in degrees,
// 0 = north, increasing clockwise (matches getInertialHeading()).
//
// HOW TO USE:
//   1. In runPreAutonomous(), after starting the odom thread, call mclInit().
//   2. Optionally call mclApply() at any time to snap odometry to MCL estimate.
//   3. Read the estimate with mclGetX(), mclGetY(), mclGetTheta() at any time.
//
// TUNING (in src/mcl.cpp):
//   MCL_SENSOR_SIGMA  - sensor noise std dev (inches). Increase if MCL jumps too much.
//   MCL_TRANS_NOISE   - fraction of translation added as noise.
//   MCL_ROT_NOISE     - fraction of rotation added as noise.
//   MCL_NUM_PARTICLES - more = more accurate but more CPU.
// ============================================================================

// Number of particles - higher = more accurate but uses more CPU/memory.
#define MCL_NUM_PARTICLES 100

// Initialize particles centered at current odometry pose, then start the
// background MCL thread. Call once after the odom thread is already running.
void mclInit();

// Initialize particles uniformly across the entire field (global localization).
// Use when starting position is completely unknown.
void mclInitUniform();

// Reset all particles to the current odometry pose (e.g. after distanceReset).
void mclReset();

// Get MCL estimated position. x/y in inches, theta in degrees.
double mclGetX();
double mclGetY();
double mclGetTheta();

// Copy the MCL estimate into x_pos, y_pos, and correct_angle.
// Use when you trust MCL more than raw odometry.
void mclApply();
