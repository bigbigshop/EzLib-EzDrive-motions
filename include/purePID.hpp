#pragma once

void drivePID(double driveDistance, double drive_kP, double drive_kI, double drive_kD, int timeout);
void turnPID(double turnDistance, double Turn_kP, double turn_kI, double turn_kD, int timeout);

