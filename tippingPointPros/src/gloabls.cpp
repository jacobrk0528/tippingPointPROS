#include "globals.hpp"

//CONTROLLER
pros::Controller Master(CONTROLLER_MASTER);
  
//MOTORS
pros::Motor rightFrontMotor(13, MOTOR_GEARSET_18, 1, MOTOR_ENCODER_COUNTS);
pros::Motor rightBackMotor(12, MOTOR_GEARSET_18, 1, MOTOR_ENCODER_COUNTS);
pros::Motor leftFrontMotor(10, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_COUNTS);
pros::Motor leftBackMotor(17, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_COUNTS);

pros::Motor frontLeftLiftMotor(19, MOTOR_GEARSET_36, 1, MOTOR_ENCODER_COUNTS);
pros::Motor frontRightLiftMotor(11, MOTOR_GEARSET_36, 0, MOTOR_ENCODER_COUNTS);
// update globals.hpp
pros::Motor backLeftLiftMotor(20, MOTOR_GEARSET_36, 0, MOTOR_ENCODER_COUNTS);
pros::Motor backRightLiftMotor(14, MOTOR_GEARSET_36, 1, MOTOR_ENCODER_COUNTS);

//PNEUMATICS
pros::ADIDigitalOut frontClawPiston('H');

//SENSORS
pros::Imu inertial(5);

pros::Rotation LOdometer(18), ROdometer(19);


