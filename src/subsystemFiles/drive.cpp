#include "main.h"
#include "globals.cpp"

void autoTurn(int degrees, int voltage) {
    int degreesTurned = 0;
    int initialIntertialRotation = (int) imu_sensor.get_rotation();

  	while (!(degreesTurned < degrees + 5) && (degreesTurned > degrees - 5)) {

    if (voltage < 0) {
        leftWheels.move_voltage(voltage);
    } else {
        rightWheels.move_voltage(voltage);
    }
    degreesTurned = initialIntertialRotation - (int) imu_sensor.get_rotation();
    pros::lcd::print(1, "IMU get rotation: %d degrees\n", (int) imu_sensor.get_rotation());
		pros::delay(20);
	}
}