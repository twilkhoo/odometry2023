#include <iostream>
#include <memory>
#include "main.h"
#include "Odom.h"
#include "motionalgos.h"
#include "portdef.h"
#include "globals.h"
#include "drive.h"

// Task Setup.

void UpdateOdometry(void* _chassis){
  while (true) {
    std::shared_ptr<Odom>* chassis = static_cast<std::shared_ptr<Odom>*>(_chassis);

    (*chassis)->updateOdom(encoderLeft.get_value(), encoderRight.get_value(), encoderBack.get_value());

    pros::delay(20);
  }
}

void PrintOdometry(void* _chassis){
  while (true) {
    std::shared_ptr<Odom>* chassis = static_cast<std::shared_ptr<Odom>*>(_chassis);

    pros::lcd::print(0, "x: %lf", (*chassis)->getX());
    pros::lcd::print(1, "y: %lf", (*chassis)->getY());
    pros::lcd::print(2, "a: %lf", (*chassis)->getHeading());

    pros::delay(20);
  }
}

void PrintEncs(){
  while (true) {
    pros::lcd::print(4, "rEnc: %d", encoderLeft.get_value());
    pros::lcd::print(5, "lEnc: %d", encoderLeft.get_value());
    pros::lcd::print(6, "bEnc: %d", encoderLeft.get_value());

    pros::delay(20);
  }
}

// Pros required config.

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

void initialize() {
	pros::lcd::initialize();
	pros::lcd::register_btn1_cb(on_center_button);

	setDriveBrakeMode(pros::E_MOTOR_BRAKE_COAST);

}
void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
  // Create the odom class.
  auto chassis = std::make_shared<Odom>(7.125, 7.125, 7.0625, 2.4125);

  pros::Task RunOdom(UpdateOdometry, static_cast<void*>(chassis.get()));
  pros::Task PrintOdom(UpdateOdometry, static_cast<void*>(chassis.get()));
  pros::Task PrintRawEnc(PrintEncs);

	while (true) {
    // This works since the V5 screen doesn't have multitouch.
    if ((pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1) {
      encoderLeft.reset();
      encoderRight.reset();
      encoderBack.reset();
      chassis->resetOdom();
    }

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
			turnToHeadingPID(chassis, 90, 50); // Example usage of a motion algo.
		}

		pros::delay(20);
	}
}
