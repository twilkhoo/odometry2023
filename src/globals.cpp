// ------- globals.cpp ---------------------------------------------------------
//
// Use globals.cpp together with globals.h to define all motor and other objects
// which should be available/accessible in autonomous and opcontrol code.
//
// forexample setup a motor definition and then add a extern pros::motor to
// globals.h to ensure all other modules have access -- of course by including
// #include "globals.h" in the relevant source (.cpp) files

#include "main.h"
#include "portdef.h"

#include <iostream>       // For file system support in C++ ability to write to file stream
#include <fstream>
#include <chrono>         // for time support - NOTE V5 has no date/time support!
#include <ctime>


// --------------------- Global Motor Definitions ------------------------------

pros::Motor driveLeftFront (LEFT_MOTOR_FRONT, MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor driveLeftBack (LEFT_MOTOR_BACK, MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor driveRightFront (RIGHT_MOTOR_FRONT, MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor driveRightBack (RIGHT_MOTOR_BACK, MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);

// -------------------- Remote Controls -----------------------------------------

pros::Controller controller(pros::E_CONTROLLER_MASTER);
//pros::Controller partner(pros::E_CONTROLLER_PARTNER);

// ------------------ IMU ------------------------------------------------------

//pros::Imu imu_sensor(IMU_PORT); //(for gyro)

// ---------------- SHAFT encoder in drive train -------------------------------
pros::ADIEncoder encoderLeft (QUAD_LEFT_TOP_PORT, QUAD_LEFT_BOTTOM_PORT);
pros::ADIEncoder encoderRight (QUAD_RIGHT_TOP_PORT, QUAD_RIGHT_BOTTOM_PORT);
pros::ADIEncoder encoderBack (QUAD_MID_TOP_PORT, QUAD_MID_BOTTOM_PORT);
