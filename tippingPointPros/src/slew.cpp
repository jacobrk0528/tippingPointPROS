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
    std::cout << "text" << std::endl;
    while(true) {
        if(pros::competition::is_autonomous() == 0) {
            double leftJoystickY = Master.get_analog(ANALOG_LEFT_Y);
            double leftJoystickX = Master.get_analog(ANALOG_LEFT_X);

            if(fabs(leftJoystickY) < 5) {
                leftJoystickY = 0;
            }
            if(fabs(leftJoystickX) < 5) {
                leftJoystickX = 0;
            }

            //set motor power
            leftFrontMotor.move_voltage(leftJoystickY*100 + leftJoystickX*100);
            leftBackMotor.move_voltage(leftJoystickY*100 + leftJoystickX*100);
            rightFrontMotor.move_voltage(leftJoystickY*100 - leftJoystickX*100);
            rightBackMotor.move_voltage(leftJoystickY*100 - leftJoystickX*100);
        }
    }
}

void DriveSlew::start(void* ignore) {
    DriveSlew *that = static_cast<DriveSlew*>(ignore);
    that -> arcadeDrive(900, 500, 900);
}
////////////// DRIVE SLEW /////////////