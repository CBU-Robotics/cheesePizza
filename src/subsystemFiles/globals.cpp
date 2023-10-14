#include "main.h"

//Initialize controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

//Initialize drivebase motors
pros::Motor leftFrontWheel(12, pros::E_MOTOR_GEARSET_18, false);
pros::Motor rightFrontWheel(1, pros::E_MOTOR_GEARSET_18, true);
pros::Motor leftBackWheel(20, pros::E_MOTOR_GEARSET_18, false);
pros::Motor rightBackWheel(10, pros::E_MOTOR_GEARSET_18, true);

//Initialize drivebase motor groups
pros::Motor_Group leftWheels({leftFrontWheel, leftBackWheel});
pros::Motor_Group rightWheels({rightFrontWheel, rightBackWheel});

//Initialize Sensors
pros::Imu imu_sensor(14);