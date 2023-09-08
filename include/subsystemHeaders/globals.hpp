#include "main.h"

//Declare controller
extern pros::Controller master;

//Declare drivebase motors
pros::Motor leftFrontWheel;
pros::Motor rightFrontWheel;
pros::Motor leftBackWheel;
pros::Motor rightBackWheel;

//Declare drivebase motor groups
extern pros::Motor_Group leftWheels;
extern pros::Motor_Group rightWheels;