#include "vex.h"

using namespace vex;
using namespace std;

extern controller Controller1;

extern motor_group LeftDrive;
extern motor_group RightDrive;

extern drivetrain DriveTrain;
extern inertial InertialSensor;

extern void vexcodeInit(void);

#include "vex.h"
#include <cmath> // For std::abs

using namespace vex;

// =======================================================
// DRIVETRAIN FUNCTIONS WITH PID 
// =======================================================

// Constants for your physical setup (Adjust as needed)
const double WHEEL_DIAMETER = 4.0; // inches
const double INCHES_PER_DEG = (WHEEL_DIAMETER * 3.14159) / 360.0;

void drivePID(double targetInches, int timeoutMs) {
    // 1. Reset drive encoders
    LeftDrive.resetPosition();
    RightDrive.resetPosition();
    
    // 2. Linear PID Constants (Tweak these for your robot's weight)
    double kP = 0.45; 
    double kI = 0.00; 
    double kD = 0.15;
    double kP_drift = 0.15; // Keeps the robot driving straight

    double targetDegrees = targetInches / INCHES_PER_DEG;
    double targetHeading = InertialSensor.heading(degrees); // Remember our original path line

    double error = targetDegrees;
    double prevError = error;
    double integral = 0;
    double derivative = 0;

    timer driveTimer;
    driveTimer.reset();

    // Loop continues until we are close to the target OR we hit the timeout limit
    while (abs(error) > 15.0 && driveTimer.time(msec) < timeoutMs) {
        
        // Calculate average distance traveled so far
        double currentPosition = (LeftDrive.position(deg) + RightDrive.position(deg)) / 2.0;
        error = targetDegrees - currentPosition;

        // PID Integral & Derivative Math
        if (abs(error) < 150) { integral += error; } else { integral = 0; }
        derivative = error - prevError;
        prevError = error;

        // Base drive power calculations (Converted to Volts)
        double lateralPower = (error * kP) + (integral * kI) + (derivative * kD);

        // Cap voltage to legal maximum system limits (-12.0V to 12.0V)
        if (lateralPower > 6.0)  lateralPower = 6.0;
        if (lateralPower < -6.0) lateralPower = -6.0;

        // Gyro Straight-Line Correction Math
        double driftError = targetHeading - InertialSensor.heading(degrees);
        if (driftError > 180)  driftError -= 360;
        if (driftError < -180) driftError += 360;
        double correction = driftError * kP_drift;

        // Spin motors dynamically based on error scale
        LeftDrive.spin(forward, lateralPower + correction, volt);
        RightDrive.spin(forward, lateralPower - correction, volt);

        wait(10, msec); 
    }

    LeftDrive.stop(brake);
    RightDrive.stop(brake);
}

void turnPID(double targetAngleDegrees, int timeoutMs)
{
    // Reset current heading to 0
    InertialSensor.setRotation(0, degrees);

    // --------------------------------
    // TUNING
    // --------------------------------
    double kP = 0.15;
    double kD = 0.04;

    double maxVoltage = 4.0;

    // Don't make tiny corrections around the target
    double stopTolerance = 1.0;

    // Must remain inside tolerance for this many loops
    int stableLoops = 0;

    double previousError = 0;

    timer turnTimer;
    turnTimer.reset();

    while (turnTimer.time(msec) < timeoutMs)
    {
        // --------------------------------
        // CURRENT ANGLE
        // --------------------------------
        double currentAngle = InertialSensor.rotation(degrees);

        // --------------------------------
        // ERROR
        // --------------------------------
        // Use the full signed rotation so targets beyond 180 degrees keep
        // their requested direction instead of taking the shorter route.
        double error = targetAngleDegrees - currentAngle;

        // --------------------------------
        // STOP
        // --------------------------------
        if (abs(error) <= stopTolerance)
        {
            stableLoops++;

            if (stableLoops >= 8)
                break;
        }
        else
        {
            stableLoops = 0;
        }

        // --------------------------------
        // DERIVATIVE
        // --------------------------------
        double derivative = error - previousError;

        previousError = error;

        // --------------------------------
        // PD CONTROL
        // --------------------------------
        double turnPower = (kP * error) + (kD * derivative);

        // --------------------------------
        // SPEED LIMIT
        // --------------------------------
        if (turnPower > maxVoltage)
            turnPower = maxVoltage;

        if (turnPower < -maxVoltage)
            turnPower = -maxVoltage;

        // --------------------------------
        // APPROACHING TARGET
        // --------------------------------
        // Reduce maximum power as we get close.
        double distance = abs(error);

        if (distance < 20)
        {
            double scale = distance / 20.0;

            // Never completely remove power
            if (scale < 0.25)
                scale = 0.25;

            turnPower *= scale;
        }

        // --------------------------------
        // DEADZONE
        // --------------------------------
        // Prevent tiny motor corrections.
        if (abs(turnPower) < 0.25)
        {
            turnPower = 0;
        }

        // --------------------------------
        // MOTOR COMMAND
        // --------------------------------
        LeftDrive.spin(forward, turnPower, volt);

        RightDrive.spin(reverse, turnPower, volt);

        wait(15, msec);
    }

    // --------------------------------
    // STOP
    // --------------------------------
    LeftDrive.stop(brake);
    RightDrive.stop(brake);
}

void autonomousControl(){
  // Example autonomous sequence
  drivePID(60, 9000); // Drive forward 12 inches with a 9-second timeout
  wait(1000, msec);      // Short pause
  drivePID(-60, 9000); // Drive forward another 12 inches with a 2-second timeout
  wait(1000, msec);      // Short pause
  drivePID(84,9000); // Drive forward 84 inches with a 9-second timeout
  wait(1000, msec);      // Short pause
  drivePID(-84, 9000); // Drive forward 84 inches with a
  wait(1000, msec);   
     // Short pause
  turnPID(-180, 9000); // Turn 90 degrees with a 9-second timeout
  drivePID(120, 9000); // Drive forward 120 inches with a 9-second timeout
  wait(1000, msec);      // Short pause
  turnPID(-180, 9000); // Turn 90 degrees with a 9-second timeout
  wait (1000, msec);      // Short pause
  drivePID(120, 9000); // Drive forward 60 inches with a
}

void driverControl(){
 int deadband = 5;
  int topSpeed = 50;
  float turnScale = 0.55;

  while (true) {

    // Get joystick values
    double leftMotorSpeed = Controller1.Axis3.position() + Controller1.Axis1.position() * turnScale;

    double rightMotorSpeed = Controller1.Axis3.position() - Controller1.Axis1.position() * turnScale;


    // Limit left motor speed
    if (leftMotorSpeed > topSpeed) {
      leftMotorSpeed = topSpeed;
    }
    else if (leftMotorSpeed < -topSpeed) {
      leftMotorSpeed = -topSpeed;
    }


    // Limit right motor speed
    if (rightMotorSpeed > topSpeed) {
      rightMotorSpeed = topSpeed;
    }
    else if (rightMotorSpeed < -topSpeed) {
      rightMotorSpeed = -topSpeed;
    }


    // Deadband
    if (abs(leftMotorSpeed) < deadband) {
      leftMotorSpeed = 0;
    }

    if (abs(rightMotorSpeed) < deadband) {
      rightMotorSpeed = 0;
    }

    LeftDrive.setVelocity(leftMotorSpeed, percent);
    RightDrive.setVelocity(rightMotorSpeed, percent);


    // R1 button = temporary brake
    if (Controller1.ButtonR1.pressing()) {

      // Stop both sides
      LeftDrive.stop(brake);
      RightDrive.stop(brake);

      wait(25, msec);

    }
    else {

      // Run the motors
      LeftDrive.spin(forward);
      RightDrive.spin(forward);
    }

    wait(25, msec);
  }
}
int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // 1. Calibrate Inertial Sensor (Robot MUST stay completely still)
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("Calibrating...");
  
  InertialSensor.calibrate();
  while (InertialSensor.isCalibrating()) {
    wait(100, msec);
  }
  
  // 2. Selection Screen Setup
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("A: Autonomous");
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("B: Driver Control");

while (true) {
    if (Controller1.ButtonA.pressing()) {
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.print("Running Auton...");
      
      autonomousControl(); // Run autonomous sequence
      
      Controller1.Screen.setCursor(2, 1);
      Controller1.Screen.print("Auton Complete.");
      break; // Exit choice loop after running auton
    }
    
    if (Controller1.ButtonB.pressing()) {
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.print("Driver Mode Active");
      
      driverControl(); // Enters infinite driver control loop
      break; 
    }
}
}