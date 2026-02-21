#include "vex.h"
#include "motor-control.h"
#include "../custom/include/autonomous.h"


// Modify autonomous, driver, or pre-auton code below


void runAutonomous() {
 int auton_selected = 1;
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
 stopChassis(coast);
 heading_correction = false;
 while (true) {
   // [-100, 100] for controller stick axis values
   ch1 = controller_1.Axis1.value();
   ch2 = controller_1.Axis2.value();
   ch3 = controller_1.Axis3.value();
   ch4 = controller_1.Axis4.value();


   // true/false for controller button presses
   l1 = controller_1.ButtonL1.pressing();
   l2 = controller_1.ButtonL2.pressing();
   r1 = controller_1.ButtonR1.pressing();
   r2 = controller_1.ButtonR2.pressing();
   button_a = controller_1.ButtonA.pressing();
   button_b = controller_1.ButtonB.pressing();
   button_x = controller_1.ButtonX.pressing();
   button_y = controller_1.ButtonY.pressing();
   button_up_arrow = controller_1.ButtonUp.pressing();
   button_down_arrow = controller_1.ButtonDown.pressing();
   button_left_arrow = controller_1.ButtonLeft.pressing();
   button_right_arrow = controller_1.ButtonRight.pressing();


   // default tank drive or replace it with your preferred driver code here:
   // arcade drive: Axis3 = forward/back, Axis1 = turn
   double forward = ch3 * 0.12;
   double turn = ch1 * 0.06;
   double left = forward + turn;
   double right = forward - turn;
   // clamp values
   if (left > 100) left = 100;
   if (left < -100) left = -100;
   if (right > 100) right = 100;
   if (right < -100) right = -100;
   driveChassis(left, right);


   if(r1){
     intake1.spin(fwd,12,volt);
     intake2.stop(hold);


   } else if(r2){
     intake1.spin(fwd, 12,volt);
     intake2.spin(fwd, 12,volt);
   } else if(l2){
     intake1.spin(reverse,12,volt);
     intake2.spin(reverse,12,volt);
   } else {
     intake1.spin(fwd,0,volt);
     intake2.spin(fwd,0,volt);
   }


static bool descore_pressed = false;
static bool descore_state = false;


if(l1 && !descore_pressed){
 descore_state=!descore_state;
 descore.set(descore_state);
}
descore_pressed = l1;


static bool midgoal_pressed = false;
static bool midgoal_state=false;




if (button_b && !midgoal_pressed){
 //midgoal.set(false);
 midgoal_state=!midgoal_state;
 midgoal.set(midgoal_state);
}
 midgoal_pressed = button_b;








   static bool scraper_pressed = false;
   static bool scraper_state = false;


  
   if (button_down_arrow && !scraper_pressed) {
     scraper_state = !scraper_state;
     scraper.set(scraper_state);
   }
   scraper_pressed = button_down_arrow;


   wait(10, msec);
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

