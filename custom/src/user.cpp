#include "vex.h"
#include "motor-control.h"
#include "../custom/include/autonomous.h"




// Modify autonomous, driver, or pre-auton code below




void runAutonomous() {
int auton_selected = 4;
switch(auton_selected) {
  case 1:
    autonskillsActual();
    break;
  case 2:
    autonskills(); //newly made
    break;
  case 3:
    rightsidelow();
    break;
  case 4:
    leftandmid();
    break;
  case 5:
    leftside7();
    break;
  case 6:
    movetwoinch();
    break;
  case 7:
    rightsidepush();
    break;
  case 8:
    rightside4push();
    break;
  case 9:
    break;
}
}




// controller_1 input variables (snake_case)
int ch1, ch2, ch3, ch4;
bool l1, l2, r1, r2;
bool button_a, button_b, button_x, button_y;
bool button_up_arrow, button_down_arrow, button_left_arrow, button_right_arrow;
int chassis_flag = 0;




void runDriver() {
   // Set brake mode to coast at the start of driver control
   stopChassis(brake);


   while (true) {
       // 1. Get Joystick Inputs (PROS scale: -100 to 100)
       double forwardInput = (double)controller_1.Axis3.value();
       double turnInput = (double)controller_1.Axis1.value();


       // 2. Curvature Drive Logic
       const double TURN_REDUCTION = 0.5;  
       const double TURN_BOOST = 0.5;      


       // Normalize to [-1.0, 1.0] for the math
       double f = forwardInput / 127.0;
       double t = turnInput / 127.0;


       // Apply scaling (Curvature Drive)
       double turnScale = 1.0 - (TURN_REDUCTION * fabs(t));
       f *= turnScale;


       double speedBoost = 1.0 + (TURN_BOOST * fabs(f));
       t *= speedBoost;


       // 3. Convert back to motor units (-127 to 127)
       int leftOutput = (int)((f + t) * 127);
       int rightOutput = (int)((f - t) * 127);


       // 4. Move the Chassis
       driveChassis(leftOutput,rightOutput);


      
  
   }
}
















void runPreAutonomous() {
  // Initializing Robot Configuration. DO NOT REMOVE!
vexcodeInit();
 // Calibrate inertial sensor
inertial_sensor.calibrate();




// Wait for the Inertial Sensor to calibrate
while (inertial_sensor.isCalibrating()) {
  wait(10, msec);
}




double current_heading = inertial_sensor.heading();
Brain.Screen.print(current_heading);
 // odom tracking
resetChassis();
if(using_horizontal_tracker && using_vertical_tracker) {
  thread odom = thread(trackXYOdomWheel);
} else if (using_horizontal_tracker) {
  thread odom = thread(trackXOdomWheel);
} else if (using_vertical_tracker) {
  thread odom = thread(trackYOdomWheel);
} else {
  thread odom = thread(trackNoOdomWheel);
}
}





