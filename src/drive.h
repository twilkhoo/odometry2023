#ifndef DRIVE_H_
#define DRIVE_H_

#include "main.h"

// Helper Functions
void setDrive(int left, int right);

// Driver Control Functions
void setTank();

void setLeftArcade();

// Braking Functions
void stopDrive();

void setDriveBrakeMode(const pros::motor_brake_mode_e_t mode);

#endif
