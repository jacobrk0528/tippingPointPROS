#include "main.h"
#include "globals.hpp"

//CONTROLLER
pros::Controller master(CONTROLLER_MASTER);

//MOTORS
pros::Motor rightFrontMotor(1, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_COUNTS);
pros::Motor rightBackMotor(2, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_COUNTS);
pros::Motor leftFrontMotor(1, MOTOR_GEARSET_18, 1, MOTOR_ENCODER_COUNTS);
pros::Motor leftBackMotor(4, MOTOR_GEARSET_18, 1, MOTOR_ENCODER_COUNTS);

pros::Motor frontLift(5, MOTOR_GEARSET_36, 0, MOTOR_ENCODER_COUNTS);
pros::Motor backLift(6, MOTOR_GEARSET_36, 0, MOTOR_ENCODER_COUNTS);

pros::Motor tilter(7, MOTOR_GEARSET_36, 0, MOTOR_ENCODER_COUNTS);

pros::Motor intake(8, MOTOR_GEARSET_18, 1, MOTOR_ENCODER_COUNTS);

//PNEUMATICS
pros::ADIDigitalOut frontClaw('A');
pros::ADIDigitalOut backClaw('B');

//SENSORS
pros::Imu inertial(15);

pros::Rotation LOdometer(18), ROdometer(19);


