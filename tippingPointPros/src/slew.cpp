#include "main.h"
#include "globals.hpp"
#include "slew.hpp"

//VARS
int Slew::leftSlewOutput = 0, Slew::rightSlewOutput = 0;
int Slew::driveMax = 0;
int Slew::leftJoystick =0, Slew::rightJoystick = 0;
int Slew::leftSide = 0, Slew::rightSide = 0;
int Slew::leftTarget = 0, Slew::rightTarget = 0;

int Slew::xSlewOutput = 0, Slew::ySlewOutput = 0;

int Slew::frontLiftOutput = 0;
int Slew::frontLiftMax = 9000/127;
int Slew::frontLiftTarget = 0;
int Slew::frontLiftUpButton = 0, Slew::frontLiftdownButton = 0;

int Slew::backLiftOutput = 0;
int Slew::backLiftMax = 9000/127;
int Slew::backLiftTarget = 0;
int Slew::backLiftUpButton = 0, Slew::backLiftdownButton = 0;

//SLEW FUNCTION
int Slew::tankDrive(double fwdAccel, double deccel, double revAccel){
    //get joystick values
    leftJoystick = master.get_analog(ANALOG_LEFT_Y);
    rightJoystick = master.get_analog(ANALOG_RIGHT_Y);
    double averageJoystick = leftJoystick/rightJoystick;

    //print values for refrence
    printf("leftTarget, leftSlewOutput, actualPower, %d %d %f\n", leftTarget, leftSlewOutput, leftFrontMotor.get_actual_velocity());
    printf("rightTarget, rightSlewlewOutput, actualPower, %d %d %f\n", rightTarget, rightSlewlewOutput, rightFrontMotor.get_actual_velocity());

    //set drive max value
    if(leftJoystick/rightJoystick < 0 || leftJoystick/rightJoystick >=2 || rightJoystick/leftJoystick < 0 || rightJoystick/leftJoystick >=2){
        driveMax =9000/127;
    } else {
        driveMax=12000/127;
    }

    //set deadzone for joystick
    if(abs(leftJoystick) <5){
        leftJoystick = 0;
    }
    if(abs(rightJoystick) <5){
        rightJoystick = 0;
    }

    //set left target
    leftTarget = leftJoystick*driveMax;

    //accelerate - if output value is less then desired, then increase output value
    if(leftSlewOutput < leftTarget){
        if(leftTarget == 0 && leftSlewOutput !=0){
            leftSlewOutput = 0;
        } else {
            leftSlewOutput +=fwdAccel;
        }
    }
    //decelerate - if output value is greater then desired, then decrease output value
    if(leftSlewOutput > leftTarget){
        if(leftTarget == 0 && leftSlewOutput !=0){
            leftSlewOutput = 0;
        } else {
            leftSlewOutput -=revAccel;
        }
    }

    //set right target
    rightTarget = rightJoystick*driveMax;

    //accelerate - if output value is less then desired, then increase output value
    if(rightSlewOutput < rightTarget){
        if(rightTarget == 0 && rightSlewOutput !=0){
            RslerightSlewOutputwOutput = 0;
        } else {
            rightSlewOutput +=fwdAccel;
        }
    }
    //decelerate - if output value is greater then desired, then decrease output value
    if(rightSlewOutput > rightTarget){
        if(rightTarget == 0 && rightSlewOutput !=0){
            rightSlewOutput = 0;
        } else {
            rightSlewOutput -=revAccel;
        }
    }

    //set motor power
    leftFrontMotor.move_voltage(leftSlewOutput);
    leftBackMotor.move_voltage(leftSlewOutput);
    rightFrontMotor.move_voltage(rightSlewOutput);
    rightBackMotor.move_voltage(rightSlewOutput);

    return 0;
}

int Slew::arcadeDrive(double fwdAccel, double deccel, double revAccel) {
    rightJoystickX = master.get_analog(ANALOG_RIGHT_X);
    rightJoystickY = master.get_analog(ANALOG_RIGHT_Y);

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
    xTarget = rightJoystickX*driveMax;

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
    yTarget = rightJoystickY*driveMax;

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

    return 0;

}

int slew::frontLift(double fwdAccel, double deccel, double revAccel) {
    frontLiftUpButton = Master.get_digital(DIGITAL_R1);
    frontLiftDownButton = Master.get_digital(DIGITAL_R2);

    printf("target, output, actual power, %d %d %f\n", frontLiftTarget, frontLiftOutput, frontLiftMotor.get_actual_velocity());

    frontTarget = frontLiftMax;

    //accelerate
    if(frontLiftUpButton > frontLiftDownButton) {
        if(frontLiftOutput < frontLiftMax) {
            if (frontLiftTarget = 0 && frontLiftOutput != 0) {
                frontLiftOutput = 0;
            } else {
                frontLiftOutput += fwdAccel;
            }
        }
    }

    //decelerate
    if(frontLiftUpButton < frontLiftDownButton) {
        if(frontLiftOutput > frontLiftMax) {
            if (frontLiftTarget = 0 && frontLiftOutput != 0) {
                frontLiftOutput = 0;
            } else {
                frontLiftOutput -= revAccel;
            }
        }
    }

    //set motor power
    frontLiftMotor.move_voltage(frontLiftOutput);

    return 0;
}

int slew::backLift(double fwdAccel, double deccel, double revAccel) {
    backLiftUpButton = Master.get_digital(DIGITAL_L1);
    backLiftDownButton = Master.get_digital(DIGITAL_L2);

    printf("target, output, actual power, %d %d %f\n", backLiftTarget, backLiftOutput, backLiftMotor.get_actual_velocity());

    backTarget = backLiftMax;

    //accelerate
    if(backLiftUpButton > backLiftDownButton) {
        if(backLiftOutput < backLiftMax) {
            if (backLiftTarget = 0 && backLiftOutput != 0) {
                backLiftOutput = 0;
            } else {
                backLiftOutput += fwdAccel;
            }
        }
    }

    //decelerate
    if(backLiftUpButton < backLiftDownButton) {
        if(backLiftOutput > backLiftMax) {
            if (backLiftTarget = 0 && backLiftOutput != 0) {
                backLiftOutput = 0;
            } else {
                backLiftOutput -= revAccel;
            }
        }
    }

    //set motor power
    backLiftMotor.move_voltage(backLiftOutput);

    return 0;
}