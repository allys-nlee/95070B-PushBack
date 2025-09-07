#include "vex.h"
#include "auton.h"
#include <iostream>
#include <string>

using namespace vex;
using namespace std;

// Drivetrain PID
void drivePID(double targetdegrees, double drivekp = 0.82, double driveki = 0, double drivekd = 1.5) {
   Inertial.setRotation(0, degrees);
   double error = targetdegrees;
   double integral = 0;
   double lasterror = error;
   double lspeed;
   double rspeed;
   double prevdegrees = FL.position(degrees);
   double startrotation = Inertial.rotation(degrees);
   double rotdif = 0;
   double krotdif = 0;
   double count = 0;

   FL.setPosition(0, degrees);
   BL.setPosition(0, degrees);
   FR.setPosition(0, degrees);
   BR.setPosition(0, degrees);
   ML.setPosition(0, degrees);
   MR.setPosition(0, degrees);

while (fabs(error) > 30) {
       double measureddegrees = (FL.position(degrees) + FR.position(degrees)) / 2;
       error = targetdegrees - measureddegrees;

       if(fabs(measureddegrees - prevdegrees) < 3){
           count++; //add to count
       } else { //if not being stalled
           count = 0;
       }

       if (count > 10) { //exit when stuck for 200 ms
           FL.stop(brake);
           FR.stop(brake);
           ML.stop(brake);
           MR.stop(brake);
           BR.stop(brake);
           BL.stop(brake);
           return;
       }

       prevdegrees = measureddegrees;

     //Integral windup
       if ((fabs((error) < targetdegrees / (10*3)) && (fabs(integral) < 300))) {
           integral += error;
       }

     if(fabs(error) < 30) {
         FL.stop(brake);
         FR.stop(brake);
         ML.stop(brake);
         MR.stop(brake);
         BR.stop(brake);
         BL.stop(brake);
         return;
       }

        // PID calculation
        double pid = error * drivekp + integral * driveki + (error - lasterror) * drivekd;
        lspeed = pid;
        rspeed = pid;

        // Heading correction
        rotdif = Inertial.rotation(degrees) - startrotation;
        krotdif = rotdif * 4;

        // Apply correction
        double lfinal = lspeed - krotdif;
        double rfinal = rspeed + krotdif;

        // Set motor directions based on target sign
        directionType dir = (targetdegrees >= 0) ? vex::forward : vex::reverse;

        FL.spin(dir, fabs(lfinal), rpm);
        ML.spin(dir, fabs(lfinal), rpm);
        BL.spin(dir, fabs(lfinal), rpm);
        FR.spin(dir, fabs(rfinal), rpm);
        MR.spin(dir, fabs(rfinal), rpm);
        BR.spin(dir, fabs(rfinal), rpm);

        lasterror = error;
        wait(20, msec);
    }
}






void slowdrivePID(double targetdegrees, double drivekp = 0.82, double driveki = 0, double drivekd = 1.5) {
   Inertial.setRotation(0, degrees);
   double error = targetdegrees;
   double integral = 0;
   double lasterror = error;
   double lspeed;
   double rspeed;
   double prevdegrees = FL.position(degrees);
   double startrotation = Inertial.rotation(degrees);
   double rotdif = 0;
   double krotdif = 0;
   double count = 0;

   FL.setPosition(0, degrees);
   BL.setPosition(0, degrees);
   FR.setPosition(0, degrees);
   BR.setPosition(0, degrees);
   ML.setPosition(0, degrees);
   MR.setPosition(0, degrees);

while (fabs(error) > 30) {
       double measureddegrees = (FL.position(degrees) + FR.position(degrees)) / 2;
       error = targetdegrees - measureddegrees;

       if(fabs(measureddegrees - prevdegrees) < 3){
           count++; //add to count
       } else { //if not being stalled
           count = 0;
       }

       if (count > 10) { //exit when stuck for 200 ms
        //    FL.stop(brake);
        //    FR.stop(brake);
        //    ML.stop(brake);
        //    MR.stop(brake);
        //    BR.stop(brake);
        //    BL.stop(brake);
           return;
       }

       prevdegrees = measureddegrees;

     //Integral windup
       if ((fabs((error) < targetdegrees / (10*3)) && (fabs(integral) < 300))) {
           integral += error;
       }

    //  if(fabs(error) < 30) {
    //     //  FL.stop(brake);
    //     //  FR.stop(brake);
    //     //  ML.stop(brake);
    //     //  MR.stop(brake);
    //     //  BR.stop(brake);
    //     //  BL.stop(brake);
    //      return;
    //    }

        // PID calculation
        double pid = error * drivekp + integral * driveki + (error - lasterror) * drivekd;
        lspeed = pid;
        rspeed = pid;

        // Heading correction
        rotdif = Inertial.rotation(degrees) - startrotation;
        krotdif = rotdif * 4;

        // Apply correction
        double lfinal = (lspeed - krotdif) * 0.5;
        double rfinal = (rspeed + krotdif) * 0.5;

        // Set motor directions based on target sign
        directionType dir = (targetdegrees >= 0) ? vex::forward : vex::reverse;

        FL.spin(dir, fabs(lfinal), rpm);
        ML.spin(dir, fabs(lfinal), rpm);
        BL.spin(dir, fabs(lfinal), rpm);
        FR.spin(dir, fabs(rfinal), rpm);
        MR.spin(dir, fabs(rfinal), rpm);
        BR.spin(dir, fabs(rfinal), rpm);

        lasterror = error;
        wait(20, msec);
    }
}






void slowturnPID(double turndegrees, double turnkp = 2.2, double turnki = 0, double turnkd = 0.01) {
    Inertial.setRotation(0, degrees);  // reset to 0
    double error = turndegrees;
    double integral = 0;
    double lasterror = error;
    double speed = 0;

    double currentdeg = Inertial.rotation(degrees);
    double prevdeg = currentdeg;
    double count = 0;

    while (fabs(error) > 2) {
        currentdeg = Inertial.rotation(degrees);
        double actualTurned = currentdeg - prevdeg;
        error = turndegrees - actualTurned;

        if (fabs(currentdeg - prevdeg) < 0.3) {
            count++;
        } else {
            count = 0;
        }

        if (count > 50) break;

        integral += error;
        speed = error * turnkp + integral * turnki + (error - lasterror) * turnkd * 0.00001;

        FL.spin(fwd, speed, rpm);
        ML.spin(fwd, speed, rpm);
        BL.spin(fwd, speed, rpm);
        FR.spin(fwd, -speed, rpm);
        MR.spin(fwd, -speed, rpm);
        BR.spin(fwd, -speed, rpm);

        lasterror = error;
        wait(20, msec);
    }

    FL.stop(brake);
    ML.stop(brake);
    BL.stop(brake);
    FR.stop(brake);
    MR.stop(brake);
    BR.stop(brake);
}







void slowestdrivePID(double targetdegrees, double drivekp = 0.82, double driveki = 0, double drivekd = 1.5) {
   Inertial.setRotation(0, degrees);
   double error = targetdegrees;
   double integral = 0;
   double lasterror = error;
   double lspeed;
   double rspeed;
   double prevdegrees = FL.position(degrees);
   double startrotation = Inertial.rotation(degrees);
   double rotdif = 0;
   double krotdif = 0;
   double count = 0;

   FL.setPosition(0, degrees);
   BL.setPosition(0, degrees);
   FR.setPosition(0, degrees);
   BR.setPosition(0, degrees);
   ML.setPosition(0, degrees);
   MR.setPosition(0, degrees);

while (fabs(error) > 30) {
       double measureddegrees = (FL.position(degrees) + FR.position(degrees)) / 2;
       error = targetdegrees - measureddegrees;

       if(fabs(measureddegrees - prevdegrees) < 3){
           count++; //add to count
       } else { //if not being stalled
           count = 0;
       }

       if (count > 10) { //exit when stuck for 200 ms
        //    FL.stop(brake);
        //    FR.stop(brake);
        //    ML.stop(brake);
        //    MR.stop(brake);
        //    BR.stop(brake);
        //    BL.stop(brake);
           return;
       }

       prevdegrees = measureddegrees;

     //Integral windup
       if ((fabs((error) < targetdegrees / (10*3)) && (fabs(integral) < 300))) {
           integral += error;
       }

    //  if(fabs(error) < 30) {
    //     //  FL.stop(brake);
    //     //  FR.stop(brake);
    //     //  ML.stop(brake);
    //     //  MR.stop(brake);
    //     //  BR.stop(brake);
    //     //  BL.stop(brake);
    //      return;
    //    }

        // PID calculation
        double pid = error * drivekp + integral * driveki + (error - lasterror) * drivekd;
        lspeed = pid;
        rspeed = pid;

        // Heading correction
        rotdif = Inertial.rotation(degrees) - startrotation;
        krotdif = rotdif * 4;

        // Apply correction
        double lfinal = (lspeed - krotdif) * 0.35;
        double rfinal = (rspeed + krotdif) * 0.35;

        // Set motor directions based on target sign
        directionType dir = (targetdegrees >= 0) ? vex::forward : vex::reverse;

        FL.spin(dir, fabs(lfinal), rpm);
        ML.spin(dir, fabs(lfinal), rpm);
        BL.spin(dir, fabs(lfinal), rpm);
        FR.spin(dir, fabs(rfinal), rpm);
        MR.spin(dir, fabs(rfinal), rpm);
        BR.spin(dir, fabs(rfinal), rpm);

        lasterror = error;
        wait(20, msec);
    }
}








void turnPID(double turndegrees, double turnkp = 2.2, double turnki = 0, double turnkd = 0.01) {
    Inertial.setRotation(0, degrees);  // reset to 0
    double error = turndegrees;
    double integral = 0;
    double lasterror = error;
    double speed = 0;

    double currentdeg = Inertial.rotation(degrees);
    double prevdeg = currentdeg;
    double count = 0;

    while (fabs(error) > 2) {
        currentdeg = Inertial.rotation(degrees);
        double actualTurned = currentdeg - prevdeg;
        error = turndegrees - actualTurned;

        if (fabs(currentdeg - prevdeg) < 0.3) {
            count++;
        } else {
            count = 0;
        }

        if (count > 50) break;

        integral += error;
        speed = error * turnkp + integral * turnki + (error - lasterror) * turnkd;

        FL.spin(fwd, speed, rpm);
        ML.spin(fwd, speed, rpm);
        BL.spin(fwd, speed, rpm);
        FR.spin(fwd, -speed, rpm);
        MR.spin(fwd, -speed, rpm);
        BR.spin(fwd, -speed, rpm);

        lasterror = error;
        wait(20, msec);
    }

    FL.stop(brake);
    ML.stop(brake);
    BL.stop(brake);
    FR.stop(brake);
    MR.stop(brake);
    BR.stop(brake);
}





//inchestodegrees code for drivePID
double inchestodegrees(double inches){
   return (inches / (3.25 * M_PI)) * 360.0;
}

//random auton intake controls
void intakeUse() {
     intake.spin(vex::forward, 12000, voltageUnits::mV);
     storage.spin(vex::reverse, 9000, voltageUnits::mV);
     outake.spin(vex::reverse, 4000, voltageUnits::mV);
}

void outtakeUse() {
     intake.spin(vex::reverse, 12000, voltageUnits::mV);
     storage.spin(vex::forward, 7000, voltageUnits::mV);
     outake.spin(vex::reverse, 4000, voltageUnits::mV);
}

void longGoal() {
     intake.spin(vex::forward, 12000, voltageUnits::mV);
     storage.spin(vex::forward, 7000, voltageUnits::mV);
     outake.spin(vex::forward, 12000, voltageUnits::mV);
}

void intakeStop() {
     intake.stop(brake);
     storage.stop(brake);
     outake.stop(brake);
}

void centertopUse() {
     intake.spin(vex::forward, 12000, voltageUnits::mV);
     storage.spin(vex::forward, 10000, voltageUnits::mV);
     outake.spin(vex::reverse, 4000, voltageUnits::mV);
}

void hardstop() {
     FL.stop(brake);
     FR.stop(brake);
     ML.stop(brake);
     MR.stop(brake);
     BR.stop(brake);
     BL.stop(brake);
}

void auton1(){
    slowdrivePID(inchestodegrees(32));
    wait(30, msec);
    slowturnPID(19);
    intakeUse();
    slowestdrivePID(inchestodegrees(20));
    slowturnPID(-28);
    intakeStop();
    turnPID(-45);
    slowdrivePID(inchestodegrees(-40));
    turnPID(-124);
    mlm.set(true);
    drivePID(inchestodegrees(19));
    intakeUse();
    slowdrivePID(inchestodegrees(3));
    drivePID(inchestodegrees(3));
    wait(150, msec);
    mlm.set(false);
    slowdrivePID(inchestodegrees(-41));
    longGoal();
}
void auton2(){
    slowdrivePID(inchestodegrees(35));
    wait(30, msec);
    slowturnPID(-32);
    intakeUse();
    slowestdrivePID(inchestodegrees(15));
    turnPID(32);
    drivePID(inchestodegrees(8.5));
    intakeStop;
    turnPID(-135);
    wait(2000, msec);
    drivePID(inchestodegrees(-11.5));
    centertopUse();
    wait(2000, msec);
    intakeStop();
    drivePID(inchestodegrees(70));
    slowturnPID(-51);
    mlm.set(true);
    intakeUse();
    wait(1500, msec);
    drivePID(inchestodegrees(19));
}
void auton3(){
}
void auton4(){
}
