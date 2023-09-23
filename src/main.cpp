#include "main.h"

const int TOP_LEFT_MOTOR_PORT = 11;
const int TOP_RIGHT_MOTOR_PORT = 1;
const int BOTTOM_LEFT_MOTOR_PORT = 20;
const int BOTTOM_RIGHT_MOTOR_PORT = 10;
const int VEX_MAX_VOLTAGE = 12000; // Don't use
const int MAX_VOLTAGE = VEX_MAX_VOLTAGE - 2000;

void initialize() {
	pros::lcd::initialize();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor top_left_motor(TOP_LEFT_MOTOR_PORT, pros::E_MOTOR_GEAR_200);
	pros::Motor top_right_motor(TOP_RIGHT_MOTOR_PORT, pros::E_MOTOR_GEAR_200);
	pros::Motor bottom_left_motor(BOTTOM_LEFT_MOTOR_PORT, pros::E_MOTOR_GEAR_200);
	pros::Motor bottom_right_motor(BOTTOM_RIGHT_MOTOR_PORT, pros::E_MOTOR_GEAR_200);

	pros::Motor_Group left_group({ top_left_motor, bottom_left_motor });
	pros::Motor_Group right_group({ top_right_motor, bottom_right_motor });

	left_group.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
	right_group.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);

	while (true) {
		int x = master.get_analog(ANALOG_RIGHT_X);
		int y = master.get_analog(ANALOG_RIGHT_Y);

		// x+ left ++ right --
		// x- left -- right ++
		// y+ left ++ right ++
		// y- left -- right --

		if (x == 0) {
			left_group.move_voltage(0);
		}
		else {
			left_group.move_voltage(x > 0 ? MAX_VOLTAGE : -MAX_VOLTAGE);
		}

		pros::lcd::print(0, "%d %d %d", x, y);
		pros::delay(20);
	}
}
