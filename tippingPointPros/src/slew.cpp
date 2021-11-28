#include "slew.hpp"

////////////// DRIVE SLEW /////////////
//VARS
int DriveSlew::leftSlewOutput = 0, DriveSlew::rightSlewOutput = 0;
int DriveSlew::driveMax = 0;
int DriveSlew::leftJoystick = 0, DriveSlew::rightJoystick = 0;
int DriveSlew::leftSide = 0, DriveSlew::rightSide = 0;
int DriveSlew::leftTarget = 0, DriveSlew::rightTarget = 0;

int DriveSlew::xSlewOutput = 0, DriveSlew::ySlewOutput = 0;

//SLEW FUNCTION
void DriveSlew::arcadeDrive(double fwdAccel, double deccel, double revAccel) {
    double rightJoystickX = Master.get_analog(ANALOG_RIGHT_X);
    double rightJoystickY = Master.get_analog(ANALOG_RIGHT_Y);

    //Set drive max
    driveMax = 11000/127;

    //Set deadzone
    if (abs(rightJoystickX) > 5) {
        rightJoystickX = 0;
    }
    if (abs(rightJoystickY) > 5) {
        rightJoystickY = 0;
    }

    //set x target
    double xTarget = rightJoystickX*driveMax;

    //accelerate - if output value is less then desired, then increase output value
    if(xSlewOutput < xTarget){
        if(xTarget == 0 && xSlewOutput !=0){
            xSlewOutput = 0;
        } else {
            xSlewOutput +=fwdAccel;
        }
    }
    //decelerate - if output value is greater then desired, then decrease output value
    if(xSlewOutput > xTarget){
        if(xTarget == 0 && xSlewOutput !=0){
            xSlewOutput = 0;
        } else {
            xSlewOutput -=revAccel;
        }
    }

    //set y target
    double yTarget = rightJoystickY*driveMax;

    //accelerate - if output value is less then desired, then increase output value
    if(ySlewOutput < yTarget){
        if(yTarget == 0 && ySlewOutput !=0){
            ySlewOutput = 0;
        } else {
            ySlewOutput +=fwdAccel;
        }
    }
    //decelerate - if output value is greater then desired, then decrease output value
    if(ySlewOutput > yTarget){
        if(yTarget == 0 && ySlewOutput !=0){
            ySlewOutput = 0;
        } else {
            ySlewOutput -=revAccel;
        }
    }

    //set motor power
    leftFrontMotor.move_voltage(ySlewOutput + xSlewOutput);
    leftBackMotor.move_voltage(ySlewOutput + xSlewOutput);
    rightFrontMotor.move_voltage(ySlewOutput - xSlewOutput);
    rightBackMotor.move_voltage(ySlewOutput - xSlewOutput);
}
void DriveSlew::start(void* ignore) {
    DriveSlew *that = static_cast<DriveSlew*>(ignore);
    that -> arcadeDrive(900, 500, 900);
}
////////////// DRIVE SLEW /////////////