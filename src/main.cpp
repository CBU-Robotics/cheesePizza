#include "main.h"

void initialize() {
	pros::lcd::initialize();
	
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

int sgn(int val) {
    if (val > 0) return 1;
    if (val < 0) return -1;
    return 0;
}


//struct for containing slew rate information for any motors that need to use it
typedef struct {
  int motorValue;
  float slewRate;
  float slewThreshold;
} slewController;

//program time counter for calculating delta T
int timeCounter = pros::millis();
int deltaT;


/**
 * Create a slew controller for a motor
 *
 * @param port  the motor port to use as a number (0 - 10)
 *
 * returns a new and configured slew controller strucutre
**/
slewController 
slewFactory (float slewRate, float slewThreshold) {
  slewController s;
  s.slewRate = slewRate;
  s.slewThreshold = slewThreshold;
  return s;
}


/**
 * Update the output value of a slew rate controller
 *
 * @param driveSet  the desired speed state
 * @param s  the slew controller structure to adjust
 *
 * returns the updated slew controller structure
 **/
slewController
setSlew (int driveSet, slewController s) {
  if (driveSet == s.motorValue) {
    return s;
  }

  if (fabs (driveSet - s.motorValue) < s.slewThreshold || fabs (driveSet - s.motorValue) <= 1) {
    s.motorValue = driveSet;
  } else {
    float motorAdd = (float) s.slewRate * (float) deltaT / 1000.0;
    if (fabs (motorAdd) < 1) {
      //always ramp by 1 so that the output doesn't get stuck
      motorAdd = 1;
    }
    
    int sign = sgn(driveSet - s.motorValue);
    s.motorValue += sign * motorAdd;
  } 

  return s;
}

void opcontrol() {
  /* Configure slew controllers to have a rate of 
     12.7 units per second with a threshold of 30 units.
  */
  slewController leftSlew = slewFactory (12.70, 30);
  slewController rightSlew = slewFactory (12.70, 30);

  while (true) {
    //keep at start of while loop, this keeps track of time each loop takes
    deltaT = pros::millis() - timeCounter;
    timeCounter = pros::millis();

    int driveLeftSet = master.get_analog(ANALOG_LEFT_Y);
    int driveRightSet = master.get_analog(ANALOG_RIGHT_Y);

    leftSlew = setSlew (driveLeftSet, leftSlew);
    rightSlew = setSlew (driveRightSet, rightSlew);

    if (master.is_connected()) {
        leftWheels = leftSlew.motorValue;
        rightWheels = rightSlew.motorValue;
    }
    pros::delay(5);
  }
}