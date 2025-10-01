#include "vex.h"
#include "auton.h"
#include <iostream>
#include <string>

using namespace vex;
using namespace std;

// Drivetrain PID
void drivePID(double targetdegrees, double drivekp = 0.36 /*0.7*/, double driveki = 0.0014 /*0.0*/ /*hesitation*/, double drivekd = 0.3 /*0.67*/ /*increase next time*/ ) {
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
   printf("target: %f degrees\n", targetdegrees);
   FL.setPosition(0, degrees);
   BL.setPosition(0, degrees);
   FR.setPosition(0, degrees);
   BR.setPosition(0, degrees);
   ML.setPosition(0, degrees);
   MR.setPosition(0, degrees);

while (fabs(error) > 3) {
       double measureddegrees = (FL.position(degrees) + FR.position(degrees)) / 2;
       error = targetdegrees - measureddegrees;
       if(fabs(measureddegrees - prevdegrees) < 2){
           count++; //add to count
       } else { //if not being stalled
           count = 0;
       }

       if (count > 40) { //exit when stuck for 400 ms
           FL.stop(brake);
           FR.stop(brake);
           ML.stop(brake);
           MR.stop(brake);
           BR.stop(brake);
           BL.stop(brake);
           printf("exit1\n");
           return;
       }

       prevdegrees = measureddegrees;

    //Integral windup
       if ((fabs((error) < targetdegrees / (10*3)) && (fabs(integral) < 300))) {
           integral += error;
       }

       if(fabs(error) < 3) {
         FL.stop(brake);
         FR.stop(brake);
         ML.stop(brake);
         MR.stop(brake);
         BL.stop(brake);
         BR.stop(brake);
         printf("exit2\n");
         return;
       }

        // PID calculation
        double pid = error * drivekp + integral * driveki + (error - lasterror) * drivekd;
        printf("FL: %f degrees, error %f %f %f %f\n", FL.position(deg), error, error * drivekp, integral * driveki, (error - lasterror) * drivekd);
        lspeed = pid;
        rspeed = pid;

        // // Heading correction
        // rotdif = Inertial.rotation(degrees) - startrotation;
        // krotdif = rotdif * 4;

        // Apply correction
        double lfinal = lspeed; //- krotdif;
        double rfinal = rspeed; //+ krotdif;

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
    printf("whileexit\n");
    double measureddegrees = (FL.position(degrees) + FR.position(degrees)) / 2;
    error = targetdegrees - measureddegrees;
    printf("final: %f degrees, error %f\n", measureddegrees, error);
    FL.stop(brake);
    FR.stop(brake);
    ML.stop(brake);
    MR.stop(brake);
    BR.stop(brake);
    BL.stop(brake);
}


void slowdrivePID(double targetdegrees, double drivekp = 0.67, double driveki = 0.0017 /*hesitation*/, double drivekd = 0.55 /*increase next time*/ ) {
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
//    printf("target: %f degrees\n", targetdegrees);
   FL.setPosition(0, degrees);
   BL.setPosition(0, degrees);
   FR.setPosition(0, degrees);
   BR.setPosition(0, degrees);
   ML.setPosition(0, degrees);
   MR.setPosition(0, degrees);

while (fabs(error) > 0.5) {
       double measureddegrees = (FL.position(degrees) + FR.position(degrees)) / 2;
       error = targetdegrees - measureddegrees;
       if(fabs(measureddegrees - prevdegrees) < 2){
           count++; //add to count
       } else { //if not being stalled
           count = 0;
       }

       if (count > 40) { //exit when stuck for 400 ms
           FL.stop(brake);
           FR.stop(brake);
           ML.stop(brake);
           MR.stop(brake);
           BR.stop(brake);
           BL.stop(brake);
        //    printf("exit1\n");
           return;
       }

       prevdegrees = measureddegrees;

    //Integral windup
       if ((fabs((error) < targetdegrees / (10*3)) && (fabs(integral) < 300))) {
           integral += error;
       }

       if(fabs(error) < 2) {
         FL.stop(brake);
         FR.stop(brake);
         ML.stop(brake);
         MR.stop(brake);
         BL.stop(brake);
         BR.stop(brake);
        //  printf("exit2\n");
         return;
       }

        // PID calculation
        double pid = (error * drivekp + integral * driveki + (error - lasterror) * drivekd) * 0.5;
        // printf("FL: %f degrees, error %f %f %f %f\n", FL.position(deg), error, error * drivekp, integral * driveki, (error - lasterror) * drivekd);
        lspeed = pid;
        rspeed = pid;

        // // Heading correction
        // rotdif = Inertial.rotation(degrees) - startrotation;
        // krotdif = rotdif * 4;

        // Apply correction
        double lfinal = lspeed; //- krotdif;
        double rfinal = rspeed; //+ krotdif;

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
    // printf("whileexit\n");
    double measureddegrees = (FL.position(degrees) + FR.position(degrees)) / 2;
    error = targetdegrees - measureddegrees;
    // printf("final: %f degrees, error %f\n", measureddegrees, error);
    FL.stop(brake);
    FR.stop(brake);
    ML.stop(brake);
    MR.stop(brake);
    BR.stop(brake);
    BL.stop(brake);
}


void slowturnPID(double turndegrees, double turnkp = 2.8, double turnki = 0, double turnkd = 0.005) {
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
        speed = (error * turnkp + integral * turnki + (error - lasterror) * turnkd) * 0.6;

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


void skillsdrivePID(double targetdegrees, double drivekp = 0.82, double driveki = 0, double drivekd = 1.5) {
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
        double lfinal = (lspeed - krotdif) * 0.2;
        double rfinal = (rspeed + krotdif) * 0.2;

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


void turnPID(double turndegrees, double turnkp = 2.1, double turnki = 0.0001, double turnkd = 1.2) {
    Inertial.setRotation(0, degrees);  // reset to 0
    double error = turndegrees;
    double integral = 0;
    double lasterror = error;
    double speed = 0;
    // printf("target: %f degrees\n", turndegrees);
    double currentdeg = Inertial.rotation(degrees);
    double prevdeg = currentdeg;
    double count = 0;

    while (fabs(error) > 1) {
        currentdeg = Inertial.rotation(degrees);
        double actualTurned = currentdeg - prevdeg;
        error = turndegrees - actualTurned;
        // printf("deg: %f degrees, error %f %f %f %f\n", currentdeg, error, error * turnkp, integral * turnki, (error - lasterror) * turnkd);
        if (fabs(currentdeg - prevdeg) < 0.3) {
            count++;
        } else {
            count = 0;
        }

        if (count > 20) { //exit after 400 msec
            // printf("stuck\n");
            break;
        }
        integral += error;
        speed = error * turnkp + integral * turnki + (error - lasterror) * turnkd;
        // printf("%f %f %f\n", error * turnkp, integral * turnki, (error - lasterror) * turnkd);

        FL.spin(fwd, speed, rpm);
        ML.spin(fwd, speed, rpm);
        BL.spin(fwd, speed, rpm);
        FR.spin(fwd, -speed, rpm);
        MR.spin(fwd, -speed, rpm);
        BR.spin(fwd, -speed, rpm);

        lasterror = error;
        wait(20, msec);
    }
    // printf("whileexit\n");
    FL.stop(brake);
    ML.stop(brake);
    BL.stop(brake);
    FR.stop(brake);
    MR.stop(brake);
    BR.stop(brake);
    wait(200, msec);
    currentdeg = Inertial.rotation(degrees);
    // printf("lastdeg %f\n", currentdeg);
}





//inchestodegrees code for drivePID
double inchestodegrees(double inches){
   return (inches / (3.25 * M_PI)) * 360.0;
}

//random auton intake controls
void intakeUse() {
     intake.spin(vex::forward, 12000, voltageUnits::mV);
     storage.spin(vex::reverse, 12000, voltageUnits::mV);
     outake.spin(vex::reverse, 12000, voltageUnits::mV);
}

void outtakeUse() {
     intake.spin(vex::reverse, 12000, voltageUnits::mV);
     storage.spin(vex::forward, 7000, voltageUnits::mV);
     outake.spin(vex::reverse, 4000, voltageUnits::mV);
}

void longGoal() {
     intake.spin(vex::forward, 12000, voltageUnits::mV);
     storage.spin(vex::forward, 8000, voltageUnits::mV);
     outake.spin(vex::forward, 10000, voltageUnits::mV);
}

void intakeStop() {
     intake.stop(brake);
     storage.stop(brake);
     outake.stop(brake);
}

void centertopUse() {
     intake.spin(vex::forward, 12000, voltageUnits::mV);
     storage.spin(vex::forward, 10000, voltageUnits::mV);
     outake.spin(vex::reverse, 3000, voltageUnits::mV);
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
    // drivePID(inchestodegrees(20));
    drivePID(inchestodegrees(32));
    wait(30, msec);
    turnPID(23);
    intakeUse();
    slowdrivePID(inchestodegrees(22));
    turnPID(-78);
    intakeStop();
    slowdrivePID(inchestodegrees(-40));
    turnPID(-122);
    drivePID(inchestodegrees(-25));
    longGoal();
    wait(2000, msec);
    mlm.set(true);
    intakeUse();
    slowdrivePID(inchestodegrees(33));
    drivePID(inchestodegrees(9));
    wait(300, msec);
    slowdrivePID(inchestodegrees(-35));
    drivePID(inchestodegrees(-15));
    longGoal();
}
void auton2(){
    slowdrivePID(inchestodegrees(32));
    wait(30, msec);
    slowturnPID(-23);
    intakeUse();
    slowdrivePID(inchestodegrees(22));
    slowturnPID(78);
    intakeStop();
    slowdrivePID(inchestodegrees(-42));
    slowturnPID(120);
    slowdrivePID(inchestodegrees(-25));
    longGoal();
    wait(2000, msec);
    mlm.set(true);
    intakeUse();
    slowdrivePID(inchestodegrees(37));
    drivePID(inchestodegrees(9));
    wait(90, msec);
    slowdrivePID(inchestodegrees(-30));
    slowdrivePID(inchestodegrees(-20));
    longGoal();
}
void auton3(){
    slowdrivePID(inchestodegrees(32));
    wait(30, msec);
    slowturnPID(-21);
    intakeUse();
    slowdrivePID(inchestodegrees(22));
    slowturnPID(30);
    drivePID(inchestodegrees(9));
    intakeStop;
    turnPID(-142);
    wait(20, msec);
    drivePID(inchestodegrees(-9.5));
    wait(20, msec);
    centertopUse();
    wait(2000, msec);
    intakeStop();
    slowdrivePID(inchestodegrees(60));
    slowturnPID(-48);
    slowdrivePID(inchestodegrees(-32));
    mlm.set(true);
}
void auton4(){
    skillsdrivePID(inchestodegrees(32));
    wait(30, msec);
    slowturnPID(-24);
    intakeUse();
    slowdrivePID(inchestodegrees(18));
    slowdrivePID(inchestodegrees(5));
    slowturnPID(85);
    intakeStop();
    skillsdrivePID(inchestodegrees(-38));
    slowturnPID(112);
    mlm.set(true);
    drivePID(inchestodegrees(-13));
    skillsdrivePID(inchestodegrees(-11));
    longGoal();
    wait(1000, msec);
    intakeUse();
    wait(1000, msec);
    longGoal();
    wait(3600, msec);
    intakeUse();
    slowdrivePID(inchestodegrees(21));
    slowdrivePID(inchestodegrees(20));
    wait(20, msec);
    slowdrivePID(inchestodegrees(-3));
    outtakeUse();
    wait(20, msec);
    intakeUse();
    slowdrivePID(inchestodegrees(3));
    wait(20, msec);
    slowdrivePID(inchestodegrees(-3));
    wait(20, msec);
    slowdrivePID(inchestodegrees(3));
    wait(4000, msec);
    turnPID(-3);
    skillsdrivePID(inchestodegrees(-22));
    skillsdrivePID(inchestodegrees(-28));
    longGoal();
    mlm.set(false);
    intakeUse();
    wait(900, msec);
    longGoal();
    wait(4000, msec);
    intakeStop();
    slowdrivePID(inchestodegrees(5));
    slowturnPID(-45);
    skillsdrivePID(inchestodegrees(54));
    slowturnPID(-56);
    slowdrivePID(inchestodegrees(-10));
    slowturnPID(9);
    outtakeUse();
    slowdrivePID(inchestodegrees(13));
    mlm.set(true);
    wait(200, msec);
    drivePID(inchestodegrees(57));
    wait(1000, msec);
    mlm.set(false);
    intakeUse();
    drivePID(inchestodegrees(-4));
}
void auton5(){
    slowdrivePID(inchestodegrees(32));
    wait(30, msec);
    slowturnPID(21);
    intakeUse();
    slowdrivePID(inchestodegrees(22));
    slowturnPID(-30);
    drivePID(inchestodegrees(9));
    intakeStop;
    turnPID(-38);
    wait(20, msec);
    drivePID(inchestodegrees(22));
    wait(20, msec);
    outtakeUse();
    wait(2000, msec);
    intakeStop();
    drivePID(inchestodegrees(-60));
}
