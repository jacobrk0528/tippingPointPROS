#include "globals.hpp"
#include "main.h"
#include "subsystems/drivebase.hpp"

bool driveBase::isSettled = true;
bool driveBase::justPID = false;
double driveBase::IMUHeading = inertial.get_heading();
double driveBase::power;
int driveBase::drive_theta;
double driveBase::kP_drive, driveBase::kD_drive, driveBase::kP_turn, driveBase::kD_turn;
int driveBase::direction_turn;
double driveBase::rate_drive, driveBase::rate_turn, driveBase::correction_rate;
double driveBase::output = 1;
int driveBase::slew_a = 600, driveBase::slew_x = 1;
int driveBase::tol, driveBase::heading_diff;
double driveBase::prevError = 0;
bool driveBase::oneSide;
double driveBase::turnPrevError = 0;
bool driveBase::halt;
double driveBase::m_error, driveBase::m_integral, driveBase::m_derivative, driveBase::m_prevError, driveBase::m_power, driveBase::LOutput, driveBase::ROutput, driveBase::drive_tol = 10, driveBase::turn_tol = 1, driveBase::t_error, driveBase::t_integral, driveBase::t_derivative, driveBase::t_prevError, driveBase::theta, driveBase::turn_kP, driveBase::turn_kI, driveBase::turn_kD, driveBase::turn_output;


driveBase::driveBase() {}
driveBase::~driveBase() {
    reset();
}

void reset() {
    rightFrontMotor.move_velocity(0);
    rightBackMotor.move_velocity(0);
    leftFrontMotor.move_velocity(0);
    leftBackMotor.move_velocity(0);
    rightFrontMotor.tare_position();
    rightBackMotor.tare_position();
    leftFrontMotor.tare_position();
    leftBackMotor.tare_position();

    inertial.reset();

    while(inertial.is_calibrating()) {
        pros::delay(5);
    }
}

void odomReset(){
    
}

void setBreak(int breakState){
    switch (breakState) {
        case 1:
            
    }
}

void stop() {

}

driveBase& withTurnSlew(int turnSlewRate = 5) {

}

driveBase& withTurnPD(double kP, double kD) {

}

driveBase& withTurnDirection(int turnDirection) {

}

driveBase& withSlew(int slewRate = 5) {

}

driveBase& withPD(double kP, double kD) {

}

driveBase& withHeading(double driveAngle, double correctionRate) {

}

driveBase& justPD(bool justPD_) {

}

driveBase& calcTurnDirection(int currentPos, int targetPos) {

}

driveBase& turn(double desiredTurnAngle) {

}

driveBase& drive(double target) {

}

driveBase& driveDistAway(double dist) {

}

void waitUntilSettled(bool halt_ = 1){

}

driveBase& withSlop(double drive_tol_ = 10, double turn_tol_ = 1){

}

driveBase& withTurn(double theta_, double turn_kP_, double turn_kI_, double turn_kD_) {

}

driveBase& move(double target, double drive_kP, double drive_kI, double drive_kD) {

}
