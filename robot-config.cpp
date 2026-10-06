#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// Brain for printing
brain Brain;

// Controller
controller Controller1 = controller(primary);
inertial InertialSensor = inertial(PORT3);

// Drivetrain motors
motor LeftMotorA = motor(PORT1, ratio18_1, true);   // front-left
motor LeftMotorB = motor(PORT9, ratio18_1, true);   // back-left

motor RightMotorA = motor(PORT4, ratio18_1, false);   // front-right (reversed)
motor RightMotorB = motor(PORT8, ratio18_1, false);  // back-right (reversed)

// Motor groups
motor_group LeftDrive = motor_group(LeftMotorA, LeftMotorB);
motor_group RightDrive = motor_group(RightMotorA, RightMotorB);

// Drivetrain object
drivetrain DriveTrain = drivetrain(LeftDrive, RightDrive);


// Initialization
void vexcodeInit(void) {
  // nothing to initialize
}
