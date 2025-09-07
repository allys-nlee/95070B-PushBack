#include "vex.h"
#include "robot-config.h"
using namespace vex;
using signature = vision::signature;
using code = vision::code;

brain  Brain;

controller Controller1 = controller(primary);
motor FL = motor(PORT1, ratio6_1, true);
motor ML = motor(PORT2, ratio6_1, true);
motor BL = motor(PORT3, ratio6_1, true);
motor FR = motor(PORT4, ratio6_1, false);
motor MR = motor(PORT5, ratio6_1, false);
motor BR = motor(PORT6, ratio6_1, false);

motor intake = motor(PORT8, ratio18_1, false);
motor outake = motor(PORT9, ratio18_1, false);
motor storage = motor(PORT7, ratio6_1, false);

pneumatics mlm = pneumatics(Brain.ThreeWirePort.A);

inertial Inertial = inertial(PORT10);
//color ColorSensor = color(PORT5);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/*
Used to initialize code/tasks/devices added using tools in VEXcode Pro.
This should be called at the start of your int main function.
*/


void vexcodeInit( void ) {
// Nothing to initialize
}