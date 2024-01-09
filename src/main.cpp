#include "main.h"
#include "subsystemFiles/globals.cpp"
#include "subsystemFiles/drive.cpp"
#include <vector>
#include <iostream>

const int TOP_LEFT_MOTOR_PORT = 12;
const int TOP_RIGHT_MOTOR_PORT = 1;
const int BOTTOM_LEFT_MOTOR_PORT = 20;
const int BOTTOM_RIGHT_MOTOR_PORT = 9;

const int INTERTIAL_SENSOR_PORT = 4;
const int VEX_MAX_VOLTAGE = 12000; // Don't use
const int MAX_VOLTAGE = VEX_MAX_VOLTAGE - 4000;
const int ANALOG_MAX_VALUE = 127;
const double INTERPOLATION_MAGNITUDE = 0.01;
const int INTERPOLATION_ERROR = 30;
const double pi = 3.14159265358979323846;
const int IMU_PORT = 14;

//The amount the left side needs to catch up to the right
const double offset = 1.04;

// Controller and motor setup
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor top_left_motor(TOP_LEFT_MOTOR_PORT, pros::E_MOTOR_GEAR_200, false, MOTOR_ENCODER_ROTATIONS);
pros::Motor top_right_motor(TOP_RIGHT_MOTOR_PORT, pros::E_MOTOR_GEAR_200, true, MOTOR_ENCODER_ROTATIONS);
pros::Motor bottom_left_motor(BOTTOM_LEFT_MOTOR_PORT, pros::E_MOTOR_GEAR_200, false, MOTOR_ENCODER_ROTATIONS);
pros::Motor bottom_right_motor(BOTTOM_RIGHT_MOTOR_PORT, pros::E_MOTOR_GEAR_200, true, MOTOR_ENCODER_ROTATIONS);

// Motor groups and brake modes
pros::Motor_Group left_group({ top_left_motor, bottom_left_motor });
pros::Motor_Group right_group({ top_right_motor, bottom_right_motor });

// Encoder
pros::ADIEncoder encoder ('A', 'B', MOTOR_ENCODER_ROTATIONS);

void move_distance(double voltage, double diameter, double distance) {
	// need to delay on start up
	encoder.reset();

	while((abs((int)encoder.get_value())) < distance / (diameter * pi)) {
		left_group.move_voltage(voltage);
		right_group.move_voltage(voltage);
	}
	left_group.brake();
	right_group.brake();
}

void move(int voltage, double diameter, double distance) {
	// need to delay on start up
	double top_left = top_left_motor.get_position();
	double top_right = top_right_motor.get_position();
	double bottom_left = bottom_left_motor.get_position();
	double bottom_right = bottom_right_motor.get_position();
	double average = 0;

	while(abs(average) < distance / (pi * diameter)) {
		double tl_dif = top_left_motor.get_position() - top_left;
		double tr_dif = top_right_motor.get_position() - top_right;
		double bl_dif = bottom_left_motor.get_position() - bottom_left;
		double br_dif = bottom_right_motor.get_position() - bottom_right;
		average = (tl_dif + tr_dif + bl_dif + br_dif) / 4;
		
		left_group.move_voltage(voltage * offset);
		right_group.move_voltage(voltage);
	}
	left_group.brake();
	right_group.brake();
}

// Sensors
pros::Imu imu_sensor(14);


bool autoTurn(int degrees, int voltage) {
    int degreesTurned = 0;
    int initialIntertialRotation = (int) imu_sensor.get_rotation();
  	while (!((degreesTurned < degrees + 3) && (degreesTurned > degrees - 3))) {
      pros::lcd::print(2, "Degrees turned: %d\n", degreesTurned);
      degreesTurned = abs(initialIntertialRotation - (int) imu_sensor.get_rotation());
    if (voltage > 0) {
        left_group.move_voltage(voltage);
    } else {
        right_group.move_voltage(-voltage);
    }
    pros::lcd::print(3, "IMU get rotation: %d degrees\n", (int) imu_sensor.get_rotation());
		pros::delay(20);
	}
  left_group.brake();
  right_group.brake();
  return false;
}

void turn (int voltage, double diameter, int rotation, double length, double width) {
	// need to delay on start up
	double top_left = top_left_motor.get_position();
	double top_right = top_right_motor.get_position();
	double bottom_left = bottom_left_motor.get_position();
	double bottom_right = bottom_right_motor.get_position();
	double average = 0;

	double percent = rotation / 360;
	double big_diameter = sqrt(pow(length, 2) + pow (width, 2));
	double circumference = pi * big_diameter;
	double rotation_distance = percent * circumference / 2.25;
	double distance = rotation_distance * sqrt(1 + pow(length / width, 2));

	while(average < distance / (pi * diameter)) {
		double tl_dif = top_left_motor.get_position() - top_left;
		double tr_dif = top_right_motor.get_position() - top_right;
		double bl_dif = bottom_left_motor.get_position() - bottom_left;
		double br_dif = bottom_right_motor.get_position() - bottom_right;
		average = (abs(tl_dif) + abs(tr_dif) + abs(bl_dif) + abs(br_dif)) / 4;
		
		left_group.move_voltage(voltage * offset);
		right_group.move_voltage(-voltage);
	}

	left_group.brake();
	right_group.brake();
}

/**
 * @brief Linearly interpolates between two values.
 *
 * This function performs linear interpolation (lerp) between an initial value
 * and a target value based on a specified magnitude.
 *
 * @param initial_value The starting value.
 * @param target_value The target value to interpolate towards.
 * @param magnitude The magnitude of interpolation, typically in the range [0, 1].
 * @return The interpolated value as a 32-bit signed integer.
 */
std::int32_t lerp(std::uint32_t initial_value, std::int32_t target_value, double magnitude) {
	return static_cast<std::int32_t>((target_value - initial_value) * magnitude) + initial_value;
}

/**
 * @brief Linearly interpolates motor group voltage to a target value.
 *
 * Performs linear interpolation (lerp) of the voltage for a given motor group
 * towards a specified target voltage using a set interpolation magnitude.
 *
 * @param motor_group A reference to the `pros::Motor_Group` object to control.
 * @param target_voltage The desired target voltage for the motor group.
 */
void interpolate_motor_voltage(pros::Motor_Group& motor_group, std::int32_t target_voltage) {
	std::vector<std::uint32_t> voltages = motor_group.get_voltages();

	if (abs(voltages[0] - target_voltage) < INTERPOLATION_ERROR) {
		motor_group.move_voltage(target_voltage);
	}
	else {
		motor_group.move_voltage(lerp(voltages[0], target_voltage, INTERPOLATION_MAGNITUDE));
	}
}

void initialize() {
	pros::lcd::initialize();
	left_group.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
	right_group.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	pros::delay(5000);
	turn(5000, 4, 360, 10.6, 10.2);
	/*
	while (true) {
// Joystick input
		int x = master.get_analog(ANALOG_RIGHT_X);
		int y = master.get_analog(ANALOG_RIGHT_Y);

		if (x == 0 && y == 0) {
			// Stop motors if joystick is at the center
			// interpolate_motor_voltage(left_group, 0);
			// interpolate_motor_voltage(right_group, 0);
			left_group.move_voltage(0);
			right_group.move_voltage(0);
			pros::lcd::print(0, "%d %d %d", 0, 0, 0); // LCD display
		}
		else {
			// Calculate magnitude based on normalized x and y
			double normalized_x = static_cast<double>(x) / 127.0;
			double normalized_y = static_cast<double>(y) / 127.0;
			double magnitude = sqrt(normalized_x * normalized_x + normalized_y * normalized_y);

			// Calculate motor voltages based on joystick input
			double angle = atan2(y, x);
			double voltage_x = cos(angle) * MAX_VOLTAGE * magnitude;
			double voltage_y = sin(angle) * MAX_VOLTAGE * magnitude;

			// Distribute voltages for forward/backward and turning
			int voltage_left = voltage_y + voltage_x;
			int voltage_right = voltage_y - voltage_x;

			// Apply interpolated motor voltages
			// interpolate_motor_voltage(left_group, voltage_left);
			// interpolate_motor_voltage(right_group, voltage_right);
			left_group.move_voltage(voltage_left);
			right_group.move_voltage(voltage_right);

			// LCD display for debugging
			pros::lcd::print(0, "%d %d %d", static_cast<int>(angle), static_cast<int>(voltage_x), static_cast<int>(voltage_y));
		}

		pros::delay(20); // Delay for loop iteration
	}
	*/
}