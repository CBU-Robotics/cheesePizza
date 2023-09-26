#include "main.h"

const int TOP_LEFT_MOTOR_PORT = 11;
const int TOP_RIGHT_MOTOR_PORT = 1;
const int BOTTOM_LEFT_MOTOR_PORT = 20;
const int BOTTOM_RIGHT_MOTOR_PORT = 10;
const int VEX_MAX_VOLTAGE = 12000; // Don't use
const int MAX_VOLTAGE = VEX_MAX_VOLTAGE - 4000;

void initialize() {
	pros::lcd::initialize();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor top_left_motor(TOP_LEFT_MOTOR_PORT, pros::E_MOTOR_GEAR_200, false);
	pros::Motor top_right_motor(TOP_RIGHT_MOTOR_PORT, pros::E_MOTOR_GEAR_200, true);
	pros::Motor bottom_left_motor(BOTTOM_LEFT_MOTOR_PORT, pros::E_MOTOR_GEAR_200, false);
	pros::Motor bottom_right_motor(BOTTOM_RIGHT_MOTOR_PORT, pros::E_MOTOR_GEAR_200, true);

	pros::Motor_Group left_group({ top_left_motor, bottom_left_motor });
	pros::Motor_Group right_group({ top_right_motor, bottom_right_motor });

	left_group.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
	right_group.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);

	while (true) {
		int x = master.get_analog(ANALOG_RIGHT_X);
		int y = master.get_analog(ANALOG_RIGHT_Y);

		if (x == 0 && y == 0) {
			left_group.move_voltage(0);
			right_group.move_voltage(0);
			pros::lcd::print(0, "%d %d %d", 0, 0, 0);
		}
		else {
			double angle = atan2(y, x);

			// cos and sin return between -1 and 1
			double voltage_x = cos(angle) * MAX_VOLTAGE;
			double voltage_y = sin(angle) * MAX_VOLTAGE;

			int voltage_left = 0;
			int voltage_right = 0;

			// y
			voltage_left += voltage_y;
			voltage_right += voltage_y;

			// x
			voltage_left += voltage_x;
			voltage_right -= voltage_x;

			left_group.move_voltage(voltage_left);
			right_group.move_voltage(voltage_right);

			pros::lcd::print(0, "%d %d %d", static_cast<int>(angle), static_cast<int>(voltage_x), static_cast<int>(voltage_y));
		}
		pros::delay(20);
	}
}
