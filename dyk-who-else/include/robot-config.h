#pragma once
using namespace vex;

extern brain Brain;

extern controller Controller1;

extern motor FL;
extern motor ML;
extern motor BL;
extern motor FR;
extern motor MR;
extern motor BR;

extern motor intake;
extern motor outake;
extern motor storage;

extern pneumatics mlm;

extern inertial Inertial;
/**
Used to initialize code/tasks/devices added using tools in VEXcode Pro.
*
This should be called at the start of your int main function.
*/
void vexcodeInit(void);