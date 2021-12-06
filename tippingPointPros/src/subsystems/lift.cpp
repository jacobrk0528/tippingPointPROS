#include "lift.hpp"

double frontLift::kP, frontLift::kI, frontLift::kD;
double frontLift::acceptableError = 5;
double frontLift::slewValue = 0;
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

double frontLift::getAvgPos() {
    return ((frontRightLiftMotor.get_position() + frontLeftLiftMotor.get_position()) / 2);
}

frontLift& frontLift::withDirection(int direction) {
    dir = direction;
    return *this;
}

frontLift& frontLift::justPID() {
    justPID_ = true;
    return *this;
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

void frontLift::move(double target) {
    reset();
    double error;
    double prevError = 0;
    double totalError = 0;
    double derivitive;
    double output;

    double slewOutput;

    currentPos = getAvgPos();
    while (currentPos < (target - acceptableError) || currentPos > (target + acceptableError)) {
        
        currentPos = getAvgPos();

        error = target - currentPos;
        derivitive = error - prevError;
        totalError += error;

        double power = error*kP + totalError*kI + derivitive*kD;

        if(output < power && !justPID_) {
            output += slewValue;
        } else {
            output = power;
        }

        frontLeftLiftMotor.move_velocity(output*dir);
        frontRightLiftMotor.move_velocity(output*dir);

        prevError = error;
        pros::delay(2);
    }
    stop();
}

void startMove(int speed) {
        frontLeftLiftMotor.move_velocity(speed);
        frontLeftLiftMotor.move_velocity(speed);
}

void frontLift::driver() {
    setBreakType(3);
    while(true){
        if(pros::competition::is_autonomous() == 0) {
            if(Master.get_digital(DIGITAL_R1)) {
                frontLeftLiftMotor.move_velocity(200);
                frontRightLiftMotor.move_velocity(200);
            } else if (Master.get_digital(DIGITAL_R2)) {
                frontLeftLiftMotor.move_velocity(-200);
                frontRightLiftMotor.move_velocity(-200);
            } else {
                stop();
            }
        }
    }
}

void frontLift::start(void* ignore) {
    frontLift *that = static_cast<frontLift*>(ignore);
    that -> driver();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////

double backLift::kP, backLift::kI, backLift::kD;
double backLift::acceptableError = 5;
double backLift::slewValue = 0;
double backLift::currentPos;
bool backLift::justPID_ = false;
int backLift::dir = UP;


void backLift::reset() {
    backLeftLiftMotor.move_velocity(0);
    backLeftLiftMotor.tare_position();

    backRightLiftMotor.move_velocity(0);
    backRightLiftMotor.tare_position();
}

void backLift::stop() {
    backRightLiftMotor.move_velocity(0);
    backLeftLiftMotor.move_velocity(0);
}

void backLift::setBreakType(int type) {
    switch (type) {
        case 1:
            backLeftLiftMotor.set_brake_mode(MOTOR_BRAKE_BRAKE);
            backLeftLiftMotor.set_brake_mode(MOTOR_BRAKE_BRAKE);
        case 2:
            backLeftLiftMotor.set_brake_mode(MOTOR_BRAKE_COAST);
            backLeftLiftMotor.set_brake_mode(MOTOR_BRAKE_COAST);
        default:
            backLeftLiftMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
            backRightLiftMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
    }
}

double backLift::getAvgPos() {
    return ((fabs(backLeftLiftMotor.get_position()) + fabs(backRightLiftMotor.get_position())) / 2);
}

backLift& backLift::withDirection(int direction) {
    dir = direction;
    return *this;
}

backLift& backLift::justPID() {
    justPID_ = true;
    return *this;
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

void backLift::move(double target) {
    reset();
    double error;
    double prevError = 0;
    double totalError = 0;
    double derivitive;
    double output;

    double slewOutput;

    currentPos = getAvgPos();
    while (currentPos < (target - acceptableError) || currentPos > (target + acceptableError)) {
        
        currentPos = getAvgPos();

        error = target - currentPos;
        derivitive = error - prevError;
        totalError += error;

        double power = error*kP + totalError*kI + derivitive*kD;

        if(output < power && !justPID_) {
            output += slewValue;
        } else {
            output = power * dir;
        }

        backLeftLiftMotor.move_velocity(output);
        backRightLiftMotor.move_velocity(output);

        prevError = error;
        pros::delay(2);

        printf("error: %f; currentPos: %f; output: %f;\n" , error, currentPos, output);

        if(currentPos > (target - acceptableError) && currentPos < (target + acceptableError)) {
            break;
        }
    }
    stop();
}

void backLift::startMove(int speed) {
        backLeftLiftMotor.move_velocity(speed);
        backRightLiftMotor.move_velocity(speed);
}

void backLift::driver() {
    setBreakType(3);
    while(true){
        if(pros::competition::is_autonomous() == 0) {
            if(Master.get_digital(DIGITAL_L1)) {
                backLeftLiftMotor.move_velocity(200);
                backRightLiftMotor.move_velocity(200);
            } else if (Master.get_digital(DIGITAL_L2)) {
                backLeftLiftMotor.move_velocity(-200);
                backRightLiftMotor.move_velocity(-200);
            } else {
                stop();
            }
        }
    }
}

void backLift::start(void* ignore) {
    backLift *that = static_cast<backLift*>(ignore);
    that -> driver();
}