#include "main.h"
#include "lemlib/timer.hpp"

double clamp(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

void drivePID(double driveDistance, double drive_kP, double drive_kI, double drive_kD, int timeout) {
  // Motor-encoder degrees, not EzDrive inches. Holds the chassis motion slot
  // for the whole loop so this cannot write the drive at the same time as ezPID.
  if (!chassis.acquireMotion()) return;
  chassis.setDistTraveled(0);

  float currentTime = pros::c::millis();
  double error = 0;      // How far the robot is from the target distance
  double integral = 0;   // Area under the error vs time graph
  double derivative = 0; // Slope of the error vs time graph
  double prevError = 0;  // The error for the previous iteration of the PID loop
  //timer1.clear();

  // Motor power variables
  double motorPower = 0;     // Power to apply to the motors
  double prevMotorPower = 0; // The motor power for the previous iteration

  // Reset motor encoders
    leftMotors.tare_position();
    rightMotors.tare_position();

  while (true) {
    // Calculate the current distance of the robot
    //printf("heading: %f\n", imu.rotation(deg));
    double currentDistance = (leftMotors.get_position() + rightMotors.get_position())/ 2;
    error = driveDistance - currentDistance; // Calculate distance error

    // PID calculations
    derivative = error - prevError; // Calculate the derivative term
    motorPower = clamp(((drive_kP * error) + (drive_kI * integral) + (drive_kD * derivative))/100 * 127,-127,127); // Calculate motor power
    integral += error; // Update the integral

    // Adjust motor power based on heading correction
    // Apply heading correction to left and right motor powers
    double leftMotorPower = motorPower;
    double rightMotorPower = motorPower;

    // Clamp the motor power values
    // leftMotorPower = clamp(leftMotorPower, -6.0, 6.0);
    // rightMotorPower = clamp(rightMotorPower, -6.0, 6.0);

    // Spin the drive motors at the calculated velocity
    leftMotors.move(leftMotorPower);
    rightMotors.move(rightMotorPower);

    printf("error: %f, time: %f\n", error, pros::c::millis() - currentTime);

    // Exit the PID loop if within acceptable range
    if ((error > -3.0 && error < 3.0) || pros::c::millis() - currentTime > timeout) {
      break;
    }

    // Update previous variables
    prevMotorPower = motorPower;
    prevError = error;
    pros::delay(20); // Don't hog CPU
  }

  // Stop the motors when the PID is done
    leftMotors.move(0);
    rightMotors.move(0);
    chassis.setDistTraveled(-1);
    chassis.releaseMotion();
}

void turnPID(double turnDistance, double Turn_kP, double turn_kI, double turn_kD, int timeout){
  // IMU degrees, not EzDrive. Same motion slot as drivePID / ezPID / chassis motions.
  if (!chassis.acquireMotion()) return;
  chassis.setDistTraveled(0);

    //PID constants
  //imu.resetRotation();
  float currentTime = pros::c::millis();
  double count = 0;
  //Variables for turn PID
  double error = 0;      //how far the robot is from the target, in degrees
  double integral = 0;   //area under the error vs time graph
  double derivative = 0; //slope of the error vs time graph
  double prevError = 0;  //the error for the previous iteration of the PID loop

  //Motor power variables
  double motorPower = 0;     //how much power to apply to the motors, ranging from -1 (clockwise at full power) to 1 (counterclockwise at full power)
  double prevMotorPower = 0; //the motor power for the previous iteration of the PID loop
  //double startDistance = imu.rotation(degrees); 
  while(true) {
        //Calculate the current distance of the robot and store it as a number (double)

    double currentDistance = imu.get_rotation();
    //printf("currentDistance: %f\n", currentDistance); 
    pros::delay(20);

    error = turnDistance - currentDistance; //calculate error
    derivative = error - prevError; //calculate the derivative term
    motorPower = clamp(((Turn_kP * error) + (turn_kI * integral) + (turn_kD * derivative))/100 * 127,-127,127); //calculate motor 
    integral += error; //updated the integral

    leftMotors.move(motorPower);
    rightMotors.move(-motorPower);

        if (error < 0.3 && error > -0.3){
     count++;
    }
    if ((count >= 5) || pros::c::millis() - currentTime > timeout){
      break;
    }
    printf("error: %f, heading: %f\n", error, currentDistance);
    //update "previous" variables
    prevMotorPower = motorPower;
    prevError = error;
    pros::delay(20);//don't hog CPU, this stays at the end of the loop
  }
  //stop the motors when the PID is done
    leftMotors.move(0);
    rightMotors.move(0);
    chassis.setDistTraveled(-1);
    chassis.releaseMotion();
  //driveDistance is how far the robot should turn, in degrees (positive is counterclockwise)
}