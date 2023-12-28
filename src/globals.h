#ifndef GLOBALS_H_
#define GLOBALS_H_

#include "main.h"

#define GLOBAL_DELAY 250

// ------------ make sure all motors are available to all code -------------
extern pros::Motor driveLeftFront;
extern pros::Motor driveLeftBack;

extern pros::Motor driveRightFront;
extern pros::Motor driveRightBack;


// ---------- make sure we set objects for the remote controls ------------
extern pros::Controller controller;
//extern pros::Controller partner;

// --------- make sure we got IMU available ------------------------------
//extern pros::Imu imu_sensor;

// --------- shaft encoders drive train -----------------------------------
extern pros::ADIEncoder encoderLeft;
extern pros::ADIEncoder encoderRight;
extern pros::ADIEncoder encoderBack;

#endif
