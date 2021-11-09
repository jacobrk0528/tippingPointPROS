#include "main.h"

#include "class/control/slew.hpp"

//VARS
int Slew::leftSlewlewOutput =0, Slew::rightSlewOutput =0;
int Slew::driveMax =0;
int Slew::leftJoystick =0, Slew::rightJoystick =0;
int Slew::leftSide =0, Slew::rightSide=0;
int Slew::leftTarget =0, Slew::rightTarget =0;

//SLEW FUNCTION
int Slew::tankDrive(double fwdAccel, double deccel, double revAccel){
    //get joystick values
    leftJoystick = master.get_analog(ANALOG_LEFT_Y);
    rightJoystick = master.get_analog(ANALOG_RIGHT_Y);
    double averageJoystick = leftJoystick/rightJoystick;

    //print values for refrence
    printf("leftTarget, leftSlewlewOutput, actualPower, %d %d %f\n", leftTarget, leftSlewlewOutput, leftFrontMotor.get_actual_velocity());
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
    if(leftSlewlewOutput < leftTarget){
        if(leftTarget == 0 && leftSlewlewOutput !=0){
            leftSlewlewOutput = 0;
        } else {
            leftSlewlewOutput +=fwdAccel;
        }
    }
    //decelerate - if output value is greater then desired, then decrease output value
    if(leftSlewlewOutput > leftTarget){
        if(leftTarget == 0 && leftSlewlewOutput !=0){
            leftSlewlewOutput = 0;
        } else {
            leftSlewlewOutput -=revAccel;
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
    leftFrontMotor.move_voltage(leftSlewlewOutput);
    leftBackMotor.move_voltage(leftSlewlewOutput);
    rightFrontMotor.move_voltage(rightSlewOutput);
    rightBackMotor.move_voltage(rightSlewOutput);

    return 0;
}