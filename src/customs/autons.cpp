#include "main.h"
#include "api.h"
#include "customs/fsm.hpp"

void stopMotors(){
  //auto_init_universal(0.0,USE_BACK | USE_LEFT); // start_theta_field = 0 (facing north)
  //pros::delay(100);
  intake_motors.move(0);
  low_roller.move(0);
  high_roller.move(0);
}

void leftGeneral(){
  //go to matchload
  arm.extend();
  hook.extend();
  ezPID.drive(-1900, 100, 1500);
  ezPID.turn(-90,127,500);
  matchload.extend();
  pros::delay(150);

  //rush into
  chassis.tank(-40,-40);
  setIntakeState(IntakeState::AUTON_INTAKE);
  pros::delay(1000);
  chassis.cancelMotion();

  ezPID.drive(200,100,700);
  ezPID.turn(-90,100,500);

  //reset
  chassis.setPose(getRealPose(0,16.92,dist_left,-1),getRealPose(0,9.21,dist_back,-1),0);
  pros::delay(50);
  matchload.retract();
  setIntakeState(IntakeState::AUTON_INTAKE);

  //go to long goal
  chassis.moveToPose(0.45,29.81,-2,1000,{true,3,0.4,127},false);
  setIntakeState(IntakeState::L2);
  pros::delay(2000);
  setIntakeState(IntakeState::STOP);

  //backward and reset
  chassis.moveToPose(-0.47,4.58,1.04,1000,{false,8,0.3,127},false);
  ezPID.turn(0,100,300);
  pros::delay(50);
  chassis.setPose(getRealPose(0,17.85,dist_left,-1),getRealPose(0,12.35,dist_back,-1),0);
  pros::delay(50);

  //middle goal
  ezPID.turn(43,100,700);
  setIntakeState(IntakeState::AUTON_INTAKE);
  //chassis.moveToPose(40,47,40,2300,{true,2,0.2,100},false);
  ezPID.drive(3100,75,3000);
  pros::delay(800);
  setIntakeState(IntakeState::ALL_INTAKE);
  pros::delay(2300);
  setIntakeState(IntakeState::STOP);
  ezPID.drive(180, 100, 800);
  chassis.moveToPose(15.7,21.34,0,1000,{false,3,0.3,120},false);
  /*
  ezPID.turn(0,90,800);
  pros::delay(50);
  chassis.setPose(getRealPose(0,32.03,dist_left,-1),getRealPose(0,32.17,dist_back,-1),0);
  pros::delay(50);
  */

  //hook
  hook.retract();
  ezPID.turn(-12,100,500);
  ezPID.drive(1280,60,2000);
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  pros::delay(5000);
  return;





  //ezPID.drive(-400, 127, 1000);



  return;
}









void leftAuton(){
  leftGeneral();
  return;
  ezPID.drive(-1250, 127, 1000);
  ezPID.turn(-45,127,1000);
  arm.set_value(false);
  setIntakeState(IntakeState::INTAKE);
  ezPID.drive(3130, 115, 2000);
  setIntakeState(IntakeState::L2);
  pros::delay(2000);
  arm.set_value(true);
  ezPID.drive(-2356, 127, 2000);
  ezPID.turn(-84.5,127,1000);
  ezPID.turn(-90,127,1000);
  hook.set_value(false);
  ezPID.drive(100, 127, 2000);
  ezPID.drive(2000, 127, 2000);
}
void leftAutonFinal(){
  leftGeneral();
  /*pros::delay(2300);
  stopMotors();
  ezPID.turn(-281.86,100,700);
  arm.set_value(false);
  ezPID.drive(-315, 100, 700); 
  ezPID.turn(-226.45,100,700);
  ezPID.drive(450, 127, 1500); */
}


void rightAuton(){
  //go to matchload
  arm.extend();
  hook.extend();
  ezPID.drive(1700, 127, 900);
  ezPID.turn(-90,127,500);
  matchload.extend();
  pros::delay(200);
 

  //rush into
  chassis.tank(-40,-40);
  setIntakeState(IntakeState::AUTON_INTAKE);
  pros::delay(1200);
  chassis.cancelMotion();

  ezPID.drive(200,127,700);
  ezPID.turn(-90,127,500);
  
  //reset
  pros::delay(200);
  double newY = getRealPose(0,8.7,dist_back,-1);
  printf("newY: %f\n", newY);
  chassis.setPose(getRealPose(0,17.61,dist_right,1),0,0);
  pros::delay(50);
  setIntakeState(IntakeState::AUTON_INTAKE);
 
  //go to long goal
  chassis.moveToPose(2.94,30,-2.24,1300,{true,3,0.3,127},false);
  setIntakeState(IntakeState::L2);
  pros::delay(1500);
  setIntakeState(IntakeState::STOP);
  matchload.retract();


  //backward and reset
  ezPID.drive(-500,127,400);
  //hook
  ezPID.turn(43,127,500);

  //return;

  ezPID.drive(700,127,800);
  hook.retract();
  ezPID.turn(17,127,400);
  ezPID.drive(1200,127,1000);
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  pros::delay(3000);
}

void rightAutonV2(){
  chassis.moveToPose(-21.44,14.29,-133.58,1500,{true,13,0.35,127,75});
  chassis.waitUntilDone();
  right_intake.move(127);
  left_intake.move(127);
  low_roller.move(70);
  high_roller.move(50);
  chassis.moveToPose(-37.83,-1.5,-130.58,1400,{true,4,0.2,115});
  chassis.waitUntilDone();
  right_intake.move(-80);
  left_intake.move(-85);
  low_roller.move(-127);
  high_roller.move(-127);
  pros::delay(700);
  intake_motors.move(0);
  low_roller.move(0);
  high_roller.move(0);

  chassis.moveToPose(4.18,34.39,-90,1500,{false,6,0.3,127,100});
  chassis.waitUntilDone();
  arm.set_value(true);
  cover.set_value(true);
  chassis.moveToPose(-21.24,34.97,-92.16,900,{true,6,0.3,110});
  chassis.waitUntilDone();
  chassis.moveToPose(-22.24,34.97,-92.16,700,{true,6,0.3,100});
  low_roller.move(127);
  high_roller.move(127);
  pros::delay(1900);
  stopMotors();
  cover.set_value(false);
  // Was drivePID(-200, 0.35, 0.000325, 0.63, 800): -200 motor-encoder degrees.
  // inches = deg/360 * pi * 3.25 * (360 wheel rpm / 600 blue) = -3.4 in.
  // Degree-scale kP/kI/kD are not reused; this uses the gains from initialize().
  ezPID.drive(-3.4, 100, 800);
  chassis.turnToHeading(90,800);
  chassis.moveToPose(13.13,33.33,104.3,1800,{true,6,0.3,110});
  intake_motors.move(127);
  low_roller.move(127);
  high_roller.move(50);
  pros::delay(1800);
  // Was drivePID(-200, 0.35, 0.000325, 0.63, 800): -200 motor-encoder degrees -> -3.4 in.
  ezPID.drive(-3.4, 100, 800);
  stopMotors();
  chassis.turnToHeading(-90,800);
  chassis.moveToPose(-20.92,33.2,-91.97,1000,{true,4,0.3,120});
  chassis.waitUntilDone();
  cover.set_value(true);
  chassis.moveToPose(-21.92,33.2,-91.97,1000,{true,4,0.3,100});
  low_roller.move(127);
  high_roller.move(127);
  pros::delay(5000);

}

void skill(){
  //go to matchload
  arm.extend();
  hook.extend();
  ezPID.drive(-1860,120,1000);
  ezPID.turn(-90,127,1000);
  matchload.extend();
  pros::delay(100);

  //rush into
  chassis.tank(-40,-40);
  setIntakeState(IntakeState::INTAKE);
  pros::delay(1000);
  chassis.cancelMotion();
  ezPID.drive(200,127,700);
  chassis.tank(-40,-40);
  pros::delay(1500);
  chassis.cancelMotion();
  ezPID.drive(500,127,700);
  ezPID.turn(-90,127,500);

  //reset
  chassis.setPose(getRealPose(0,17.1,dist_left,-1),getRealPose(0,14.41,dist_back,-1),0);
  pros::delay(50);
  setIntakeState(IntakeState::INTAKE);


  //go to 半面場
  chassis.moveToPose(-17.32,16.09,-3.81,1000,{true,3,0.3,127,0,2},false);
  matchload.retract();
  chassis.moveToPose(-17,110,2,2300,{true,3,0.3,127},false);
  ezPID.turn(90,100,800);
  wallReset('x',-30,0,800);
  chassis.setPose(getRealPose(0,19.44,dist_left,-1),0,0);
  pros::delay(50);
  ezPID.drive(910,100,800);
  setIntakeState(IntakeState::INTAKE);
  ezPID.turn(90,100,500);

  //go to long goal
  chassis.moveToPose(18.2,19,90,1000,{true,3,0.3,127});
  chassis.waitUntil(8);
  setIntakeState(IntakeState::STOP);
  chassis.waitUntilDone();
  setIntakeState(IntakeState::L2);
  pros::delay(1200);
  setIntakeState(IntakeState::REVERSE);
  pros::delay(200);
  setIntakeState(IntakeState::L2);
  pros::delay(1200);
  setIntakeState(IntakeState::STOP);
  matchload.extend();

  //second matchloader
  chassis.moveToPose(-8.75,20.21,87.67,1000,{false,3,0.3,127});
  setIntakeState(IntakeState::L2);
  chassis.waitUntilDone();
  setIntakeState(IntakeState::INTAKE);
  chassis.tank(-40,-40);
  pros::delay(1000);
  chassis.cancelMotion();
  ezPID.drive(200,127,700);
  chassis.tank(-40,-40);
  pros::delay(1800);
  chassis.cancelMotion();

  //go to long goal
  chassis.moveToPose(18.2,19,90,1000,{true,3,0.3,127});
  chassis.waitUntil(8);
  setIntakeState(IntakeState::STOP);
  chassis.waitUntilDone();
  setIntakeState(IntakeState::L2);
  pros::delay(1200);
  setIntakeState(IntakeState::REVERSE);
  pros::delay(200);
  setIntakeState(IntakeState::L2);
  pros::delay(1200);
  setIntakeState(IntakeState::STOP);
  matchload.retract();
  setIntakeState(IntakeState::REVERSE);
  chassis.moveToPose(-0.68,34.52,190,1000,{false,6,0.6,127,0,3},false);
  chassis.moveToPose(1.52,85.32,180,1000,{false,6,0.3,127,0,2},false);
  chassis.moveToPose(-23,108,90,1000,{false,6,0.6,127,0},false);
  setIntakeState(IntakeState::STOP);
  chassis.swingToHeading(0,DriveSide::RIGHT,700);
  chassis.waitUntilDone();
  setIntakeState(IntakeState::REVERSE);

  //clear blue
  ezPID.drive(-1750,90,1400);
  ezPID.drive(2000,95,1000);
  chassis.swingToHeading(90,DriveSide::RIGHT,700);
  chassis.waitUntilDone();
  wallReset('x',-30,0,800);
  chassis.setPose(getRealPose(0,33.4,dist_left,-1),0,0);
  pros::delay(50);
  ezPID.drive(880,100,800);
  ezPID.turn(90,100,800);
  chassis.moveToPoint(-19,19.39,1000,{false,100},false);
  //arm.extend();
  ezPID.turn(0,100,600);

  //3 matchloader
  setIntakeState(IntakeState::INTAKE);
  matchload.extend();
  pros::delay(100);
  chassis.tank(-40,-40);
  pros::delay(1000);
  chassis.cancelMotion();
  ezPID.drive(200,127,700);
  chassis.tank(-40,-40);
  pros::delay(1500);
  chassis.cancelMotion();
  ezPID.drive(500,127,700);
  ezPID.turn(0,127,500);

  //reset
  chassis.setPose(getRealPose(0,17.1,dist_left,-1),getRealPose(0,14.41,dist_back,-1),0);
  pros::delay(100);
  setIntakeState(IntakeState::INTAKE);


  //go to 半面場
  chassis.moveToPose(-17.32,16.09,-3.81,1000,{true,3,0.3,127,0,2},false);
  matchload.retract();
  chassis.moveToPose(-17,110,2,2300,{true,3,0.3,127},false);
  ezPID.turn(90,100,600);
  wallReset('x',-30,0,800);
  chassis.setPose(getRealPose(0,19.44,dist_left,-1),0,0);
  pros::delay(50);
  ezPID.drive(910,100,800);
  setIntakeState(IntakeState::INTAKE);
  ezPID.turn(90,100,600);

  //go to long goal
  chassis.moveToPose(18.2,19,90,1000,{true,3,0.3,127});
  chassis.waitUntil(8);
  setIntakeState(IntakeState::STOP);
  chassis.waitUntilDone();
  setIntakeState(IntakeState::L2);
  pros::delay(1200);
  setIntakeState(IntakeState::REVERSE);
  pros::delay(200);
  setIntakeState(IntakeState::L2);
  pros::delay(1200);
  setIntakeState(IntakeState::STOP);
  matchload.extend();

  //fourth matchloader
  chassis.moveToPose(-8.75,20.21,87.67,1000,{false,3,0.3,127});
  setIntakeState(IntakeState::L2);
  chassis.waitUntilDone();
  setIntakeState(IntakeState::INTAKE);
  chassis.tank(-40,-40);
  pros::delay(1000);
  chassis.cancelMotion();
  ezPID.drive(200,127,700);
  chassis.tank(-40,-40);
  pros::delay(1500);
  setIntakeState(IntakeState::STOP);
  chassis.cancelMotion();

  //go to 4 long goal
  chassis.moveToPose(18.2,19,90,1200,{true,3,0.3,127,});
  chassis.waitUntilDone();
  setIntakeState(IntakeState::L2);
  pros::delay(1200);
  setIntakeState(IntakeState::REVERSE);
  pros::delay(200);
  setIntakeState(IntakeState::L2);
  pros::delay(1200);
  setIntakeState(IntakeState::STOP);
  matchload.retract();
  setIntakeState(IntakeState::REVERSE);
  chassis.moveToPose(-0.68,34.52,190,1000,{false,6,0.6,127,0,3},false);
  chassis.moveToPose(1.52,85.32,180,1000,{false,6,0.3,127,0,2},false);
  chassis.moveToPose(-23,108,90,1000,{false,6,0.6,127,0},false);
  setIntakeState(IntakeState::STOP);
  chassis.swingToHeading(0,DriveSide::RIGHT,700);
  chassis.waitUntilDone();
  setIntakeState(IntakeState::REVERSE);

  //clear red
  ezPID.drive(-1750,90,1400);


  pros::delay(1000);

  


  
  
  
 
  

  



}

void soloAWP(){

 arm.extend();
 hook.extend();
 ezPID.drive(1860,127,1000);
 ezPID.turn(90,127,1000);
 matchload.extend();
 pros::delay(100);


  





  pros::delay(1000);
}

void testing() {
  //chassis.moveToPoint(0,20,1000,{true,127},false);
  //ezPID.drive(12,110);
  //ezPID.drive(-12,110);
  //ezPID.turn(90,127,1000);
  chassis.moveToPose(20,20,45,1500,{true,6,0.4,127},false);
  ezPID.turn(0,127,1000);
  chassis.moveToPose(0,0,0,1500,{false,6,0.4,127,30,10});
  chassis.moveToPose(10,20,77,1500,{true,6,0.7,127},false);
   
}