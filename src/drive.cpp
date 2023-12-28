#include "main.h"
#include "globals.h"

// GENERAL DRIVE FUNCTIONS
// see drive.h
void setDrive(int left, int right) {
  driveLeftFront = left;
  driveLeftBack = left;
  driveRightFront = right;
  driveRightBack = right;
}


// DRIVER CONTROL FUNCTIONS
// see drive.h
void setTank() {
  int threshold = 10;

  int leftJoystick = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  int rightJoystick = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

  leftJoystick = (abs(leftJoystick) < threshold) ? 0 : leftJoystick;
  rightJoystick = (abs(rightJoystick) < threshold) ? 0 : rightJoystick;

  setDrive(leftJoystick, rightJoystick);
}

// see drive.h
void setLeftArcade() {
  int threshold = 10;

  int xAxis = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
  int yAxis = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

  xAxis = (abs(xAxis) < threshold) ? 0 : xAxis;
  yAxis = (abs(yAxis) < threshold) ? 0 : yAxis;

  int leftMax = abs(yAxis + xAxis) > 127 ? 127 : (yAxis + xAxis);
  int rightMax = abs(yAxis - xAxis) > 127 ? 127 : (yAxis - xAxis);

  setDrive(leftMax, rightMax);
}

// see drive.h
void stopDrive() {
  driveLeftFront.brake();
  driveLeftBack.brake();
  driveRightFront.brake();
  driveRightBack.brake();
}

// see drive.h
void setDriveBrakeMode(const pros::motor_brake_mode_e_t mode) {
  driveLeftFront.set_brake_mode(mode);
	driveLeftBack.set_brake_mode(mode);
	driveRightFront.set_brake_mode(mode);
	driveRightBack.set_brake_mode(mode);
}
