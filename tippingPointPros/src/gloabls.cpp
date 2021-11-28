#include "globals.hpp"

//CONTROLLER
pros::Controller Master(CONTROLLER_MASTER);

//MOTORS
pros::Motor rightFrontMotor(11, MOTOR_GEARSET_18, 1, MOTOR_ENCODER_COUNTS);
pros::Motor rightBackMotor(12, MOTOR_GEARSET_18, 1, MOTOR_ENCODER_COUNTS);
pros::Motor leftFrontMotor(1, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_COUNTS);
pros::Motor leftBackMotor(4, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_COUNTS);

pros::Motor frontLiftMotor(14, MOTOR_GEARSET_36, 1, MOTOR_ENCODER_COUNTS);
pros::Motor backLiftMotor(13, MOTOR_GEARSET_36, 0, MOTOR_ENCODER_COUNTS);

pros::Motor tilterMotor(5, MOTOR_GEARSET_36, 0, MOTOR_ENCODER_COUNTS);

pros::Motor intakeMotor(6, MOTOR_GEARSET_18, 1, MOTOR_ENCODER_COUNTS);

//PNEUMATICS
pros::ADIDigitalOut frontClawPiston('A');

//SENSORS
pros::Imu inertial(15);

pros::Rotation LOdometer(18), ROdometer(19);


