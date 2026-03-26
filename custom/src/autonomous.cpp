#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>
#include <thread>


#include "../include/autonomous.h"
#include "motor-control.h"



// IMPORTANT: Remember to add respective function declarations to custom/include/autonomous.h
// Call these functions from custom/include/user.cpp
// Format: returnType functionName() { code }


void movetwoinch(){
 wait(10000, msec);
 driveTo(6.5,3000);
}
void exampleAuton() {
 // Use this for tuning linear and turn pid
 driveTo(60, 3000);


 turnToAngle(90, 2000);
 turnToAngle(135, 2000);
 turnToAngle(150, 2000);
 turnToAngle(160, 2000);
 turnToAngle(165, 2000);
 turnToAngle(0, 2000);
 driveTo(-60, 3000);
}


void exampleAuton2() {
 moveToPoint(24, 24, 1, 2000, false);
 moveToPoint(48, 48, 1, 2000, true);
 moveToPoint(24, 24, -1, 2000, true);
 moveToPoint(0, 0, 1, 2000, true);
 correct_angle = 0;
 driveTo(24, 2000, false, 8);
 turnToAngle(90, 800, false);
 turnToAngle(180, 800, true);
}


void rightsidelow(){
 moveToPoint(1, -40, -1, 2500, false, 5);//drive to score low goal
 intake2.spin(fwd,5,volt);
 wait(750, msec);
 intake2.spin(fwd,0,volt);
 intake1.spin(fwd,12,volt);
 turnToPoint(-5, -36, 1, 1000);//turn to face balls
 moveToPoint(-30, -15, 1, 2000, false, 8);//drive to goal area
 turnToPoint(-30, 0, 1, 1000);//turn to face goal
 moveToPoint(-34, -37, -1, 600, false, 6);//go back and score
 intake2.spin(fwd,12,volt);
 wait(1750, msec);
 scraper.set(true); //scraper down
 intake2.spin(fwd,0,volt);
 resetChassis();
 moveToPoint(-25, 20, 1, 2250, false, 4);//drive to matchloader
 moveToPoint(-35, -37, -1, 350, false, 4);//go back and score
 intake2.spin(fwd,12,volt);
 wait(1500, msec);


 
  //moveToPoint(6.5, 25, 1, 2500, false, 5);//drive to balls
}


void rightsidepush(){
 intake1.spin(fwd,12,volt);
 turnToPoint(10, 33.5, 1, 260); //first stack, increased x coord to 9.5, 250 to 275 to 260, x coord to 10.5 to 10
 driveTo(12,2100,false,5); //distance before matchloader is put down, 13.7 to 14, then to 14.5, then to 15, then to 14, 2000 to 2100, 4 to 3.7, 13.7 to 12
 scraper.set(true); //scraper down
 wait(500,msec); // 400 to 500
 driveTo(2,500,false,7);
 moveToPoint(43.5,1,1,1500,false,10); //drive to matchload
 turnToPoint(44.2,-10,1,500);
 intake1.spin(reverse, 12, volt);
 wait(300, msec);
 intake1.spin(fwd, 12, volt);
 wait(200,msec); //added
 driveTo(17.5,1700,true,9); //drive into matchload, 14.6 changed to 14.8, then 14.7, changed to 14.5, then to 15, max voltage decreased, then to 14
 //driveChassis(30, 30); //aaron shen magic
 wait(200,msec); //wait from 270 to 235
intake1.spin(reverse, 12, volt);
intake1.spin(fwd,12,volt);
 wait(150,msec); //added
 moveToPoint(45,10.5,-1,1100,true,10); //back out of matchload, was 45.2 but changed to 45, then changed to 44, then 44.5, then 45 and 1100, then 43 to move more left from 45
 driveTo(-10, 500,true,12); // -7 to -10
 intake1.spin(reverse,12,volt);
 wait(300,msec);
 intake1.spin(fwd,12,volt);
 intake2.spin(fwd,12,volt); //score into long goal
 wait(2000,msec); // increased from 2000 to 2050 ms
 scraper.set(false);
 intake2.spin(fwd,0,volt);
 intake1.spin(fwd,0,volt);
 driveTo(15, 500, true, 12); //
 turnToAngle(-85, 400, true, 12); //increased speed
 driveTo(14.5, 850, true, 12); //13.5 to 13.9, time out increased by 50 ms from 800 to 850, DECREASED DISTANCE FROM 17 TO 16
 turnToAngle(0, 300, true, 12); //350 to 400
 driveTo(36,1300, true,8);// changed to 33 then 36, 1300
 turnToAngle(-90,9000,true,3); //-90 to -85, then to -95, then to -70, then to -10
 wait(500,msec);
}


void rightside4push(){
 intake1.spin(fwd,12,volt);
 turnToPoint(9, 33.5, 1, 4000); //first stack
 driveTo(8.6,2000,false,7);
 scraper.set(true); //scraper down
 wait(400,msec);
 driveTo(2,500,false,4);
 moveToPoint(43.7,2,1,9000,false,9); //drive to matchload
 turnToPoint(43.7,-10,1,1000);
 driveTo(-7, 1250, false, 11);
 intake2.spin(fwd, 12, volt);
 wait(1600, msec);
 scraper.set(false);
 driveTo(8, 1500, true, 12);
 turnToAngle(-90, 1000, true, 12);
 driveTo(3.5, 1000, true, 12);
 turnToAngle(0, 1000, true, 12);
 driveTo(19,9000, false,5);
}

double field_half_size = 72.0;

void leftandmid(){
 intake1.spin(fwd,12,volt);
 moveToPoint(-14,31.5,1,1400,false,6); //first stack, 31.5 to 29
 scraper.set(true);
 wait(100,msec);
 driveTo(-4.5,500,false,10); //come back to allign with midgoal
 //boomerang(-10,10,-1,-135,0.1,1500,false,8); //align with mid goal, y from 20to 10, -10 x to -14
 turnToAngle(-110, 500,false,6);
 driveChassis(-3.4,-3.4); //drive to mid goal
 wait(50, msec);
 intake2.spin(reverse, 12, volt);
 intake1.spin(reverse,12,volt);
 wait(420,msec); // before we get to the midgoal, outtake the top stage, when you getg to the midgoal outtake both for one second and then score
 intake2.spin(fwd,0,volt);
 intake1.spin(fwd,0,volt);
 wait(300,msec);
 midgoal.set(true);
 intake1.spin(fwd, 12, volt);//score
 wait(1750,msec);
 intake1.spin(fwd,0,volt);
 driveChassis(0,0);
 midgoal.set(false);
 moveToPoint(-18,0,1,1300,false,9); //drive to matchload, -20 to -10
 turnToAngle(181,500,true,5);
 scraper.set(true);
 intake1.spin(fwd,12,volt);
 wait(100, msec);
 driveTo(5,1250,false,6); //drive into matchload
 intake1.spin(fwd,12,volt);
 wait(1150,msec);
 moveToPoint(-18.4,8,-1,800,false,9); //align with long goal
 //turnToAngle(173,9000,false,4);
 driveTo(-12,800,false,11); //drive into long goal
 scraper.set(false);
 intake2.spin(fwd,0,volt);
 intake1.spin(fwd,0,volt);
 wait(700, msec);
 intake2.spin(fwd, 12, volt);
 intake1.spin(fwd, 12 ,volt);
 wait(1500,msec);
 intake2.spin(fwd,0,volt);
 intake1.spin(fwd,0,volt);
 driveTo(15, 500, true, 12);//descore
 turnToAngle(-90, 350, true, 12);
 driveTo(-11, 500, true, 12); //drive back
 turnToAngle(-180, 350, true, 12);
 driveTo(-32,1250, true,11);//descore
 turnToAngle(-90,9000,true,4);
}


void leftside7(){
 intake1.spin(fwd,12,volt);
 moveToPoint(-11.5,31,1,9000,false,4); //first stack
 turnToAngle(-160,9000,false,4);
 moveToPoint(-21,-5,1,9000,false,6); //align with long goal
 turnToAngle(180,9000,false,4);
 driveTo(-16,2000,false,10); //drive into long goal
 intake2.spin(fwd,12,volt);
 scraper.set(true);
 wait(1150,msec);
 intake2.spin(fwd,0,volt);
 moveToPoint(-20,0,1,9000,false,8); //align with matchloader
 turnToAngle(185,9000,false,8);
 wait(100,msec);
 driveChassis(6,6);
 wait(1000, msec);
 driveChassis(0,0); //drive into matchloader
 moveToPoint(-22,6,-1,9000,false,6); //align with long goal
 turnToAngle(18,9000,false,4);
 driveTo(-19,1500,false,6); //drive into long goal
 intake2.spin(fwd,12,volt);
}

void sawp (){
}

void autonskills(){// turn to first stack
  scraper.set(true);
  driveTo(59,1500,true,9);
  intake1.spin(fwd, 12, volt);
  intake2.spin(reverse, -1, volt);
  turnToAngle(-45,400,true,12);//starting angle we are at is at 135
  driveTo(23,700,true,8); 
  wait(100,msec);
  wait(1500,msec);
  turnToAngle(-80,400,true,12);
  wait(250,msec);
  turnToAngle(-65,400,true,12);
  driveTo(-27, 800, true, 9);
  turnToAngle(-45, 400, true, 12);
  driveTo(-76,1750,true,12);//corrder
  turnToAngle(45, 600, true, 12);
  driveTo(-7, 400, true, 8);
  turnToAngle(135, 600, true, 12);
  driveTo(-17, 1000, true, 9);
  intake1.spin(reverse, 12, volt);
  intake2.spin(reverse, 12, volt);
  wait(250, msec); 
  intake1.spin(fwd, 12, volt);
  intake2.spin(fwd, 12, volt);
  wait(1750, msec);
  intake2.spin(fwd,0,volt);
  turnToAngle(136, 500, false, 9);
  driveTo(40, 2000, true, 7);
  wait(1600, msec);
  turnToAngle(140, 500, true, 12);
  driveTo(-20, 1000, false, 9);
  wait(200, msec);
  turnToAngle(137, 500, true, 12);
  driveTo(-20, 1000, true, 9);
  intake1.spin(reverse, 12, volt);
  intake2.spin(reverse, 12, volt);
  wait(250, msec); 
  intake1.spin(fwd, 12, volt);
  intake2.spin(fwd, 12, volt);
  wait(1750, msec);

  //park


  driveTo(10, 500, false, 12); // starting from long goal which haas angle of 90
  turnToAngle(-45, 500, false, 12);
  driveTo(-24, 1000, false, 12);
  turnToAngle(90, 500, true, 12);


  
  

}

void autonskillsActual(){ 
  intake1.spin(fwd,12,volt);
  intake2.spin(reverse,1,volt);
  turnToAngle(60,500,true,12); // turn to face stack
  driveTo(13,750,true,8);
  intake1.spin(fwd,0,volt);
  driveTo(10,500,true,6);
  turnToAngle(-45,500,true,12); // turn to angle midgaol
  driveTo(-24,700,true,7);
  scraper.set(true);
  wait(500,msec);
  intake1.spin(reverse,12,volt); // unjam
  intake2.spin(reverse,0,volt);
  wait(100,msec);
  midgoal.set(true);
  wait(200,msec); 
  intake1.spin(fwd,10,volt); //outtake in midgoal
  wait(1000,msec);
  midgoal.set(false);
  intake2.spin(reverse,1,volt);
  driveTo(57,1500, true, 8); // drive to matchloader
  turnToAngle(-90,400,true,7); //angle to matchloader
  driveTo(23,1000,true,7); // inside of matchloader, 9 to 7 
  wait(1500,msec);
  moveToPoint(11,47,-1,1000,true,8); // hop out of matchloader, from 48 to 45 tp 49
  turnToAngle(-95,400,true,7);
  driveTo(-76,1750,true,12); //drive through corridor
  boomerang(96, 44, 1, 90, 0.3, 1000, false, 8); // align with long goal
  driveTo(-30, 1000, true, 8); //drive into it
  resetPositionLeft();
  scraper.set(true);
  intake1.spin(reverse, 12, volt);
  intake2.spin(reverse, 12, volt);
  wait(250, msec); 
  intake1.spin(fwd, 12, volt);
  intake2.spin(fwd, 12, volt);
  resetPositionLeft();
  wait(2000, msec);
  intake2.spin(fwd,0,volt);
  moveToPoint(140, 55, 1, 1100, true, 6); // 44 to 42y , drive into matchloader point
  driveTo(9.5, 600, true, 8);
  wait(1600,msec);
  //moveToPoint(76,46,-1,1000,true,8); // move back to the long goal
  turnToAngle(87,500,true,4);
  driveTo(-50,1200,true,9); //align fully to long goal
  resetPositionLeft();
  intake1.spin(reverse, 12, volt);
  intake2.spin(reverse, 12, volt);
  wait(500, msec); //unjam
  intake1.spin(fwd, 12, volt);
  intake2.spin(fwd, 12, volt);
  wait(2000,msec); //score
  intake1.spin(fwd,0,volt);
  intake2.spin(fwd,0,volt);
  scraper.set(false); //scraper up
  resetPositionLeft();
  moveToPoint(110,12,1,2700, true,8); // move to park
  turnToAngle(174,500,true,10); //align to park
  driveTo(18,1000,true,4); //move a lil further
  driveTo(-6, 300, true, 8);
  scraper.set(true); // scraper down
  wait(500,msec);
  driveTo(10, 750, true, 4);
  scraper.set(false); //scraper back up
  intake1.spin(fwd,12,volt);
  driveTo(92,2000,true,7); //drive through section
  moveToPoint(60,-28,-1,3100,true,8);
  turnToAngle(135, 400, true, 8);
  driveTo(-12.5,1000,true,9);
  
  scraper.set(true);
  intake1.spin(reverse,12,volt); // unjam
  intake2.spin(reverse,0,volt);
  wait(100,msec);
  midgoal.set(true);
  wait(200,msec); 
  intake1.spin(fwd,10,volt); //outtake in midgoal
  wait(1000,msec);
  intake1.spin(fwd, 8, volt);
  wait(500, msec);
  midgoal.set(false);
  intake2.spin(reverse,1,volt);
  /*driveTo(58,1500, true, 8); // drive to matchloader
  scraper.set(true);
  turnToAngle(90,400,true,7); //angle to matchloader
  driveTo(23,1000,true,7); // inside of matchloader, 9 to 7 
  wait(1500,msec);
  driveTo(-4,400,true,8);
  turnToAngle(70,500,true,12);
  driveTo(-25,700,true,12);
  turnToAngle(87,500,true,12);
  driveTo(-70,2000,true,12);*/


  scraper.set(true);
  driveTo(57,1500,true,9); //drive out
  intake1.spin(fwd, 12, volt);
  intake2.spin(reverse, -1, volt);
  turnToAngle(90,400,true,12); //face loader
  driveTo(23,700,true,8); // drive forward
  wait(100,msec);
  wait(1500,msec);
  turnToAngle(55,400,true,12);  //turn out
  wait(250,msec);
  turnToAngle(75,400,true,12); //
  driveTo(-29, 800, true, 9);
  turnToAngle(90, 400, true, 12);
  driveTo(-76,1750,true,12);//corrder
  turnToAngle(180, 600, true, 12);
  driveTo(-7, 400, true, 8);
  turnToAngle(-90, 600, true, 12);
  driveTo(-17, 1000, true, 9);
  intake1.spin(reverse, 12, volt);
  intake2.spin(reverse, 12, volt);
  wait(250, msec); 
  intake1.spin(fwd, 12, volt);
  intake2.spin(fwd, 12, volt);
  wait(1750, msec);
  intake2.spin(fwd,0,volt);
  turnToAngle(-89, 500, false, 9);
  driveTo(40, 2000, true, 7);
  wait(1600, msec);
  turnToAngle(-91, 500, true, 12);
  driveTo(-20, 1000, false, 9);
  wait(200, msec);
  turnToAngle(-90, 500, true, 12);
  driveTo(-20, 1000, true, 9);
  intake1.spin(reverse, 12, volt);
  intake2.spin(reverse, 12, volt);
  wait(250, msec); 
  intake1.spin(fwd, 12, volt);
  intake2.spin(fwd, 12, volt);
  wait(1750, msec);

  //park


  driveTo(10, 500, false, 12); // starting from long goal which haas angle of 90
  turnToAngle(-45, 500, false, 12);
  intake1.spin(reverse, 12, volt);
  driveTo(-24, 1000, false, 12);
  turnToAngle(0, 500, true, 12);
  driveTo(60, 2000, true, 12);



  /*
  turnToAngle(135, 500, true, 9); //midgoal angle
  moveToPoint(62, -21, -1, 2000, true, 8); //midgoal allignment
  turnToAngle(135, 1000, false, 9); //midgoal angle
  scraper.set(true);
  intake1.spin(reverse, 12, volt);
  intake2.spin(reverse, 12, volt);
  wait(300, msec);
  midgoal.set(true);
  driveTo(-6, 500, true, 9);  //  drive into midgoal
  intake2.spin(fwd, 0, volt);
  intake1.spin(fwd, 12, volt);
  wait(3500, msec);                                                                                                       
  //turnToAngle(150, 500, false, 8); // 30 to 45
  //driveTo(27, 1000, true, 9);
  // wait(3500, msec);
  // turnToAngle(150, 500, false, 8); // 30 to 45

  driveTo(27, 1000, true, 9);
 
  moveToPoint(102, -63, 1, 1500, true, 9); //align to the third loader
  midgoal.set(false);
  turnToAngle(90, 300, false, 7);
  intake1.spin(fwd, 12, volt);
  intake2.spin(reverse, 1, volt);
  scraper.set(true);
  driveTo(22.5, 1500, true, 7);  // drive into matchload
  wait(1750, msec);
  
  intake1.spin(fwd, 0, volt);
  intake2.spin(fwd, 0, volt);

  driveTo(-5,1000,false,8); // try to get in corridor
  moveToPoint(96, -82, 1, 1000, false, 8); // align to corridor
  //turnToAngle(-90, 1000, true, 9); not needed????
  moveToPoint(8, -81, 1, 3000, true, 10); //move through corridor
  wait(50, msec);

  moveToPoint(24, -60, -1, 1500, true, 8);  // align to last long goal, made negative instead of positive
  turnToAngle(-90, 700, false, 8);
  driveTo(-7, 500, false, 8); // into long goal
  scraper.set(true);
  intake1.spin(reverse, 12, volt); //unjam
  intake2.spin(reverse, 12, volt);
  wait(300, msec);
  intake1.spin(fwd,12,volt);// score
  intake2.spin(fwd,12,volt);
  wait(3000, msec);

  driveTo(33,1200,true,8); //drive into last loader
  intake1.spin(fwd,12,volt);
  intake2.spin(reverse, 1, volt);
  wait(3000,msec); // align to last long goal, made negative instead of positive
  driveTo(-33, 500, false, 8); // into long goal
  intake1.spin(reverse, 12, volt); //unjam
  intake2.spin(reverse, 12, volt);
  wait(300, msec);
  intake1.spin(fwd,12,volt);// score
  intake2.spin(fwd,12,volt);
  wait(3000,msec);

  driveTo(8, 600, false, 8);
  turnToAngle(0, 700, true, 9); //+90
  driveTo(63, 3000, false, 10);
  turnToAngle(-270, 700, false, 9);
  driveTo(-17, 1500, true, 9);
  turnToAngle(-180, 1000, false, 9);
  driveTo(5, 1000, false, 9);
  //moveToPoint(-18, 0, 1, 1000, true, 6);
  scraper.set(true);
  intake1.spin(reverse, 12, volt);
  driveTo(15, 1000, false, 11);
  scraper.set(false);
  driveTo(-15, 1000, true, 11);
  driveTo(15, 1000, true, 11);
  

  // next : align backwards to midgoal
  //then can copy paste some stuff from old autonskills
*/


  /*
  driveTo(32.5, 1000, true, 8);
  turnToAngle(-90, 400, true, 7);
  scraper.set(true);
  wait(50,msec);
  intake1.spin(fwd,12,volt);
  intake2.spin(reverse, 1, volt);
  driveTo(15.7, 1000, true, 9); // into the matchloader 
  wait(1500,msec);
  driveTo(-10, 500, true, 6); 
  wait(10, msec);
  scraper.set(false);
  descore.set(true);
  intake1.spin(fwd, 12, volt);
  moveToPoint(12, 50, 1, 1000, true, 8); //(12, 33)
  wait(10, msec); 
  turnToAngle(90, 1000, true, 6);
  driveTo(70, 3000, true, 11); // drive in the corridor
  boomerang(96, 29, 1, 180, 0.3, 1000, false, 7); // align with long goal
  turnToAngle(90, 300, false, 8);
  
  driveTo(-16, 500, true, 9); // drive into long goal
  wait(50, msec);
  
  scraper.set(true);
  intake1.spin(reverse, 12, volt); 
  intake2.spin(reverse, 12, volt); 
  wait(300, msec);
  intake1.spin(fwd, 12, volt); 
  intake2.spin(fwd, 12, volt); 
  wait(3000, msec);
  intake1.spin(fwd, 12, volt);
  intake2.spin(reverse, 1, volt);
  driveTo(33.5, 1200, true, 9); // drive into matchload //(130, 32.5)
  wait(2000, msec);
  intake1.spin(fwd, 0, volt);
  driveTo(-31, 1000, true, 8); // drive into long goal
  intake1.spin(reverse, 12, volt); 
  intake2.spin(reverse, 12, volt); 
  wait(100, msec);
  intake1.spin(fwd, 12, volt);
  intake2.spin(fwd, 12, volt);
  wait(3150, msec);

  scraper.set(false);

  intake1.spin(fwd, 12, volt);
  intake2.spin(reverse, 1, volt);
  

  driveTo(2, 400, false, 9); //come out of long goal
  turnToAngle(180, 400, false, 10);
  wait(50, msec);
  driveTo(95, 2500, false, 7);
  moveToPoint(62, -21, -1, 2000, false, 6);  //midgoal alignment
  turnToAngle(135, 1000, false, 9); //midgoal angle
  scraper.set(true);
  intake1.spin(reverse, 12, volt);
  intake2.spin(reverse, 12, volt);
  wait(300, msec);
  midgoal.set(true);
  driveTo(-6, 500, true, 9);  // drive into midgoal
  intake2.spin(fwd, 0, volt);
  intake1.spin(fwd, 12, volt);
  wait(3500, msec);                                                                                                       
  //turnToAngle(150, 500, false, 8); // 30 to 45
  //driveTo(27, 1000, true, 9);
  // wait(3500, msec);
  // turnToAngle(150, 500, false, 8); // 30 to 45

  driveTo(27, 1000, true, 9);
 
  moveToPoint(102, -63, 1, 1500, true, 9); //align to the third loader
  midgoal.set(false);
  turnToAngle(90, 300, false, 7);
  intake1.spin(fwd, 12, volt);
  intake2.spin(reverse, 1, volt);
  scraper.set(true);
  driveTo(22.5, 1500, true, 7);  // drive into matchload
  wait(1750, msec);
  
  intake1.spin(fwd, 0, volt);
  intake2.spin(fwd, 0, volt);

  driveTo(-5,1000,false,8); // try to get in corridor
  moveToPoint(96, -82, 1, 1000, false, 8); // align to corridor
  //turnToAngle(-90, 1000, true, 9); not needed????
  moveToPoint(8, -81, 1, 3000, true, 10); //move through corridor
  wait(50, msec);

  moveToPoint(24, -60, -1, 1500, true, 8);  // align to last long goal, made negative instead of positive
  turnToAngle(-90, 700, false, 8);
  driveTo(-7, 500, false, 8); // into long goal
  scraper.set(true);
  intake1.spin(reverse, 12, volt); //unjam
  intake2.spin(reverse, 12, volt);
  wait(300, msec);
  intake1.spin(fwd,12,volt);// score
  intake2.spin(fwd,12,volt);
  wait(3000, msec);

  driveTo(33,1200,true,8); //drive into last loader
  intake1.spin(fwd,12,volt);
  intake2.spin(reverse, 1, volt);
  wait(3000,msec); // align to last long goal, made negative instead of positive
  driveTo(-33, 500, false, 8); // into long goal
  intake1.spin(reverse, 12, volt); //unjam
  intake2.spin(reverse, 12, volt);
  wait(300, msec);
  intake1.spin(fwd,12,volt);// score
  intake2.spin(fwd,12,volt);
  wait(3000,msec);

  driveTo(8, 600, false, 8);
  turnToAngle(0, 700, true, 9); //+90
  driveTo(63, 3000, false, 10);
  turnToAngle(-270, 700, false, 9);
  driveTo(-17, 1500, true, 9);
  turnToAngle(-180, 1000, false, 9);
  driveTo(5, 1000, false, 9);
  //moveToPoint(-18, 0, 1, 1000, true, 6);
  scraper.set(true);
  intake1.spin(reverse, 12, volt);
  driveTo(15, 1000, false, 11);
  scraper.set(false);
  driveTo(-15, 1000, true, 11);
  driveTo(15, 1000, true, 11);


  */
  

  
 
 
}





/*
void leftside7(){
 intake1.spin(fwd,12,volt); //intake on when auton starts
 moveToPoint(-11,11,1,9000,false,11); //Moves to right before the balls
 scraper.set(true); //puts down scraper
 moveToPoint(-11.5,31,1,9000,false,4); //first stack
 scraper.set(false); //puts up scraper after intaking first set of balls
 turnToAngle(-160,9000,false,6); //idk
 moveToPoint(-21,-5,1,9000,false,8); //align with mid goal - idk
 turnToAngle(180,9000,false,6); //idk
 driveTo(-16,2000,false,6); //drive into mid goal
 intake2.spin(fwd,12,volt); //scoring into mid goal
 scraper.set(true); //putting down scraper for matchloader
 wait(1150,msec); //waiting for some reason
 intake2.spin(fwd,0,volt); //stopping intake
 moveToPoint(-35,0,1,9000,false,6); //align with matchloader (right in front of it)
 turnToPoint(-35,-100,1,9000); //turning towards matchloader
 wait(100,msec); //waiting for some reason
 intake1.spin(fwd,12,volt); //starting intake for matchloader
 driveTo(18,3000,false,6); //drive into matchloader
 wait(100,msec); //waiting for some reason
 //driveTo(-2,500,false, 6);
 //driveTo(2,500,false,6);
 moveToPoint(-35,11,-1,9000,false,8); //moving into long goal
 intake2.spin(fwd,12,volt); //scoring into long goal
 moveToPoint(-35,0,1,9000,false,10); //moving back from matchloader
 intake2.spin(fwd,0,volt); //stopping scoring intake
 intake1.spin(fwd,12,volt); //starting storing intake
 moveToPoint(35,11,1,10000,false,10); //moving to right before second set of balls
 scraper.set(true); //putting down scraper over the balls;
 moveToPoint(35,31,1,9000,false,10);
 moveToPoint(59,0,1,9000,false,10); //moving behind long goal
 moveToPoint(59,11,-1,9000,false,8); //moving inside long goal
 intake2.spin(fwd,12,volt); //scoring into long goal
}
*/


/*
void leftside(){
 descore.set(true); //descore up
 intake1.spin(fwd,12,volt); //start intake
 intake2.spin(fwd,12,volt);
 intake3.spin(reverse,12,volt);
 wait(20, msec);
 driveTo(22.5, 2500, false, 5);//drive to balls
 scraper.set(true); //scraper down
 driveTo(8, 1250, true, 5); //drive more get 3rd ball, msec);
 driveTo(-10, 1500, true); //back up
 wait(200, msec);
 scraper.set(false); //scraper up
 turnToAngle(80, 1500, true); //turn to goal
 intake1.spin(fwd,0,volt); //stop intake
 intake2.spin(fwd,0,volt);
 intake3.spin(fwd,0,volt);
 driveTo(-35, 1000, true); //back away from low goal
 turnToAngle(210, 1500, true); //turn to face goal
 driveTo(-26, 1500, false, 5); //drive to goal
 wait(500, msec);
 descore.set(false); //descore down
 wait(250, msec);
 intake1.spin(fwd,12,volt); //start intake
 intake2.spin(fwd,12,volt);
 intake3.spin(reverse,12,volt);
 wait(1250, msec);
 scraper.set(true); //scraper down
 descore.set(true); //descore up
 driveTo(25, 1000, true, 7.5); //drive to getballs
 turnToAngle(-155,1000,false);
 driveTo(25, 1000, true, 7.5);
 wait(1000, msec);
 turnToAngle(-150,1000,false);
 driveTo(-40, 1000, true,8); //back to goal
 descore.set(false); //descore down
 wait(250, msec);                   
}
*/