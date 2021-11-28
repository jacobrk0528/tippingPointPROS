#include "lift.hpp"

double frontLift::groundValue = 0;
double frontLift::kP, frontLift::kI, frontLift::kD;
double frontLift::error, frontLift::derivitive, frontLift::intagral;
double frontLift::prevError = 0;
double frontLift::output = 0, frontLift::power = 0;
int frontLift::tol = 5;
double frontLift::slewValue = 0;
double frontLift::currentPos;
bool frontLift::justPID_ = false;
int frontLift::dir = UP;


void frontLift::reset() {
    frontLiftMotor.move_velocity(0);
    frontLiftMotor.tare_position();
}

void frontLift::setBreakType(int type) {
    switch (type) {
        case 1:
            frontLiftMotor.set_brake_mode(MOTOR_BRAKE_BRAKE);
        case 2:
            frontLiftMotor.set_brake_mode(MOTOR_BRAKE_COAST);
        default:
            frontLiftMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
    }
}

void frontLift::stop() {
    frontLiftMotor.move_velocity(0);
}

void justPID() {
    bool justPID_ = true;
}

frontLift& frontLift::withPID(double kp, double ki, double kd) {
    kP = kp;
    kI = ki;
    kD = kd;
    return *this;
}

frontLift& frontLift::withSlew(int rate){
    slewValue = rate;
    return *this;
}

frontLift& frontLift::move(double target) {
    while (true) {
        currentPos = frontLiftMotor.get_position();

        if(target < currentPos) {
            dir = DOWN;
        } else if(target>currentPos) {
            dir = UP;
        } else {
            dir = NEITHER;
        }

        error = target - currentPos;
        derivitive = error - prevError;
        intagral += error;
        prevError = error;

        power = error*kP + intagral*kI + derivitive*kD;

        if(output < power && !justPID_) {
            output += slewValue;
        } else {
            output = power;
        }

        frontLiftMotor.move_voltage(output*dir);
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////

double backLift::groundValue = 0;
double backLift::kP, backLift::kI, backLift::kD;
double backLift::error, backLift::derivitive, backLift::intagral;
double backLift::prevError = 0;
double backLift::output = 0, backLift::power = 0;
int backLift::tol = 5;
double backLift::slewValue = 0;
double backLift::currentPos;
bool backLift::justPID_ = false;
int backLift::dir = UP;


void backLift::reset() {
    backLiftMotor.move_velocity(0);
    backLiftMotor.tare_position();
}

void backLift::setBreakType(int type) {
    switch (type) {
        case 1:
            backLiftMotor.set_brake_mode(MOTOR_BRAKE_BRAKE);
        case 2:
            backLiftMotor.set_brake_mode(MOTOR_BRAKE_COAST);
        default:
            backLiftMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
    }
}

void backLift::stop() {
    backLiftMotor.move_velocity(0);
}

void backLift::justPID() {
    bool justPID_ = true;
}

backLift& backLift::withPID(double kp, double ki, double kd) {
    kP = kp;
    kI = ki;
    kD = kd;
    return *this;
}

backLift& backLift::withSlew(int rate){
    slewValue = rate;
    return *this;
}

backLift& backLift::move(double target) {
    while (true) {
        currentPos = backLiftMotor.get_position();

        if(target < currentPos) {
            dir = DOWN;
        } else if(target>currentPos) {
            dir = UP;
        } else {
            dir = NEITHER;
        }

        error = target - currentPos;
        derivitive = error - prevError;
        intagral += error;
        prevError = error;

        power = error*kP + intagral*kI + derivitive*kD;

        if(output < power && !justPID_) {
            output += slewValue;
        } else {
            output = power;
        }

        backLiftMotor.move_voltage(output*dir);
    }
}

