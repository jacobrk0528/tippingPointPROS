#pragma once
#include "main.h"

//CONTROLLERS
extern pros::Controller Master;

//MOTORS
extern pros::Motor rightFrontMotor;
extern pros::Motor rightBackMotor;
extern pros::Motor leftFrontMotor;
extern pros::Motor leftBackMotor;

extern pros::Motor frontLeftLiftMotor;
extern pros::Motor frontRightLiftMotor;

extern pros::Motor backLeftLiftMotor;
extern pros::Motor backRightLiftMotor;

// PNEUMATICS
extern pros::ADIDigitalOut frontClawPiston;

//SENSORS
extern pros::Imu inertial;

extern pros::Rotation LOdometer, ROdometer;

//TASK