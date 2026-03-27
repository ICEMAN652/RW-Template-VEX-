#include <string>
#include <cmath>


// --- Global Variables (snake_case) ---
extern bool is_turning;


extern double x_pos, y_pos;
extern double correct_angle;


// --- Function Declarations (lowerCamelCase) ---
void driveChassis(double left_power, double right_power);


double getInertialHeading(bool normalize = false);
double normalizeTarget(double angle);


void turnToAngle(double turn_angle, double time_limit_msec, bool exit = true, double max_output = 12);
void driveTo(double distance_in, double time_limit_msec, bool exit = true, double max_output = 12);
void curveCircle(double result_angle_deg, double center_radius, double time_limit_msec, bool exit = true, double max_output = 12);
void swing(double swing_angle, double drive_direction, double time_limit_msec, bool exit = true, double max_output = 12);


void stopChassis(vex::brakeType type = vex::brake);
void resetChassis();
double getLeftRotationDegree();
double getRightRotationDegree();
void correctHeading();
void trackNoOdomWheel();
void trackXYOdomWheel();
void trackXOdomWheel();
void trackYOdomWheel();
void turnToPoint(double x, double y, int dir, double time_limit_msec);
void moveToPoint(double x, double y, int dir, double time_limit_msec, bool exit = true, double max_output = 12, bool overturn = false);
void boomerang(double x, double y, int dir, double a, double dlead, double time_limit_msec, bool exit = true, double max_output = 12, bool overturn = false);

extern double field_half_size;
void resetPositionWithSensor(vex::distance& sensor, double sensor_offset, double sensor_angle_deg, double field_half_size = 72);
void resetPositionFront();
void resetPositionBack();
void resetPositionLeft();
void resetPositionRight();

// Sensor descriptor for distanceReset - configure offsetX/offsetY per your robot geometry.
// offsetX: lateral offset from robot center, perpendicular to sensor pointing direction (inches)
// offsetY: distance from robot center to sensor face, along sensor pointing direction (inches)
struct DistResetSensor {
    vex::distance* sensor;  // nullptr if this sensor position is not installed
    double offsetX;
    double offsetY;
};

// Resets x and y position using distance sensors with angle correction for slight misalignment.
// xDirection: which sensor faces the x-axis wall ('F'=front, 'B'=back, 'R'=right, 'L'=left)
// yDirection: which sensor faces the y-axis wall ('F'=front, 'B'=back, 'R'=right, 'L'=left)
void distanceReset(char xDirection, char yDirection);