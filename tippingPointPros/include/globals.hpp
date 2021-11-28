#pragma once
#include "main.h"

//CONTROLLERS
extern pros::Controller Master;

//MOTORS
extern pros::Motor rightFrontMotor;
extern pros::Motor rightBackMotor;
extern pros::Motor leftFrontMotor;
extern pros::Motor leftBackMotor;

extern pros::Motor frontLiftMotor;
extern pros::Motor backLiftMotor;

extern pros::Motor tilterMotor;

extern pros::Motor intakeMotor;

// PNEUMATICS
extern pros::ADIDigitalOut forntClawPiston;
extern pros::ADIDigitalOut backClawPiston;

//SENSORS
extern pros::Imu inertial;

extern pros::Rotation LOdometer, ROdometer;
