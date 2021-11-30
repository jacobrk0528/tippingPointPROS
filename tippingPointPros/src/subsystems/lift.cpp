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
    frontLeftLiftMotor.move_velocity(0);
    frontLeftLiftMotor.tare_position();

    frontRightLiftMotor.move_velocity(0);
    frontRightLiftMotor.tare_position();
}

void frontLift::setBreakType(int type) {
    switch (type) {
        case 1:
            frontLeftLiftMotor.set_brake_mode(MOTOR_BRAKE_BRAKE);
            frontRightLiftMotor.set_brake_mode(MOTOR_BRAKE_BRAKE);
        case 2:
            frontLeftLiftMotor.set_brake_mode(MOTOR_BRAKE_COAST);
            frontRightLiftMotor.set_brake_mode(MOTOR_BRAKE_COAST);
        default:
            frontLeftLiftMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
            frontRightLiftMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
    }
}

void frontLift::stop() {
    frontLeftLiftMotor.move_velocity(0);
    frontRightLiftMotor.move_velocity(0);
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
        currentPos = frontLeftLiftMotor.get_position();

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

        frontLeftLiftMotor.move_voltage(output*dir);
        frontRightLiftMotor.move_voltage(output*dir);
    }
}

void frontLift::driver() {
    while(true){
        if(pros::competition::is_autonomous() == 0) {
            if(Master.get_digital(DIGITAL_R1)) {
                frontLeftLiftMotor.move_voltage(10000);
                frontRightLiftMotor.move_voltage(10000);
            } else if (Master.get_digital(DIGITAL_R2)) {
                frontLeftLiftMotor.move_voltage(-10000);
                frontRightLiftMotor.move_voltage(-10000);
            } else {
                frontLeftLiftMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
                frontRightLiftMotor.move_velocity(0);

                frontLeftLiftMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
                frontRightLiftMotor.move_velocity(0);
            }
        }
    }
}

void frontLift::start(void* ignore) {
    frontLift *that = static_cast<frontLift*>(ignore);
    that -> driver();
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
