/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/




// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----




#include "vex.h"
#include "auton.h"
#include <string>




using namespace vex;
using namespace std::chrono;




// A global instance of competition
competition Competition;




// matthew's uhhhh auton selector :D
bool inauton = false;
void describe(int n) {
 Controller1.Screen.setCursor(4, 1);
 if (n == 1) {
   Controller1.Screen.print("Right");
 } else if (n == 2) {
   Controller1.Screen.print("Left");
 } else if (n == 3) {
   Controller1.Screen.print("urmom");
 } else if (n == 4) {
   Controller1.Screen.print("urdad");
 }
}


int autons = 4;
int displayauton = 0;


void selectorout() {
 while (true) {
   if (Controller1.ButtonRight.pressing()) {
     displayauton++;
     wait(100, msec);
   }
   if (Controller1.ButtonLeft.pressing()) {
     displayauton--;
     wait(100, msec);
   }
   if (Controller1.ButtonA.pressing()) {
     wait(1000, msec);
     if (Controller1.ButtonA.pressing()) { //press A to select auton
       Controller1.rumble(".-.-.");
       break;
     }
   }
   if (displayauton > autons) {
     displayauton = 0;
   }
   if (displayauton < 0) {
     displayauton = autons;
   }
   if (displayauton == 0) {
     Controller1.Screen.clearScreen();
     Controller1.Screen.setCursor(1, 1);
     Controller1.Screen.print("Choose Program");
   }
   if (displayauton != 0 && inauton == false) {
     Controller1.Screen.clearScreen();
     Controller1.Screen.setCursor(1, 1);
     Controller1.Screen.print("Auton");
     Controller1.Screen.setCursor(1, 6);
     Controller1.Screen.print(displayauton);
     Controller1.Screen.setCursor(4, 1);
     describe(displayauton);
   }
 }
}


steady_clock::time_point lastmlm;
steady_clock::time_point lastaa;



/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/




void pre_auton(void) {
// Initializing Robot Configuration. DO NOT REMOVE!
vexcodeInit();
Inertial.calibrate();
Controller1.Screen.print("CALIBRATING...");




while (Inertial.isCalibrating()) {
  wait(1000, msec);
}




Controller1.Screen.clearScreen();
selectorout();
}








/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/




void autonomous(void) {
 inauton = true;




 if (displayauton == 1) {
   auton1();
 }
 if (displayauton == 2) {
   auton2();
 }
 if (displayauton == 3) {
   auton3();
 }
 if (displayauton == 4) {
   auton4();
 }


 inauton = false;
}




/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
//up

bool mlmtrue;

//away
void mlmAway(){
  mlm.set(false);
}

//use
void mlmUse() {
  mlm.set(true);
}

//use mlm
void mlmControl() {
 if (mlmtrue) {
  mlm.set(true);
 } else {
  mlm.set(false);
 }
}

//mlm using one button
void usingmlm() {
  auto now = steady_clock::now();
  auto durLastmlm = duration_cast<milliseconds>(now-lastmlm).count();
  if (durLastmlm > 200) {
    mlmControl();
    mlmtrue = !mlmtrue;
    lastmlm = now;
 }
}





bool aatrue;

void usercontrol(void) {
 while (true) {
    FL.setStopping(coast);
    ML.setStopping(coast);
    BL.setStopping(coast);
    FR.setStopping(coast);
    MR.setStopping(coast);
    BR.setStopping(coast);
   double forward = Controller1.Axis3.position() * 0.95;  // forward/backward, change multiplier for sensitivity
   double turn = Controller1.Axis1.position() * 0.75;     // left/right


   // Convert to voltage (+-12000 mV)
   double leftVoltage = (forward + turn) * 120;
   double rightVoltage = (forward - turn) * 120;


   // limmit :D
   if (leftVoltage > 12000) leftVoltage = 12000;
   if (leftVoltage < -12000) leftVoltage = -12000;
   if (rightVoltage > 12000) rightVoltage = 12000;
   if (rightVoltage < -12000) rightVoltage = -12000;


   // Drive motors (direction, side of dt, voltage units) 
   FL.spin(vex::forward, leftVoltage, voltageUnits::mV);
   ML.spin(vex::forward, leftVoltage, voltageUnits::mV);
   BL.spin(vex::forward, leftVoltage, voltageUnits::mV);


   FR.spin(vex::forward, rightVoltage, voltageUnits::mV);
   MR.spin(vex::forward, rightVoltage, voltageUnits::mV);
   BR.spin(vex::forward, rightVoltage, voltageUnits::mV);


   // intake+piston control
   //intake balls
   if (Controller1.ButtonR1.pressing()) {
     intake.spin(vex::forward, 12000, voltageUnits::mV);
     storage.spin(vex::reverse, 9000, voltageUnits::mV);
     outake.spin(vex::reverse, 4000, voltageUnits::mV);
   }
   else if (Controller1.ButtonR2.pressing()) {
     intake.spin(vex::reverse, 12000, voltageUnits::mV);
     storage.spin(vex::forward, 7000, voltageUnits::mV);
     outake.spin(vex::reverse, 4000, voltageUnits::mV);
     }
   //bottom center goal
   else if (Controller1.ButtonL1.pressing()) {
     intake.spin(vex::forward, 12000, voltageUnits::mV);
     storage.spin(vex::forward, 7000, voltageUnits::mV);
     outake.spin(vex::forward, 10000, voltageUnits::mV);
   }
   //top center goal
   else if (Controller1.ButtonL2.pressing()) {
     intake.spin(vex::forward, 12000, voltageUnits::mV);
     storage.spin(vex::forward, 7000, voltageUnits::mV);
     outake.spin(vex::reverse, 6000, voltageUnits::mV);
   }
   else if(Controller1.ButtonX.pressing()) {
    usingmlm();
   }
   else {
     intake.stop();
     storage.stop();
     outake.stop();
   }
   wait(20, msec);  // small delay for brain processing
 }
}




// Main function
int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
  while (true) {
    wait(100, msec);
  }
}