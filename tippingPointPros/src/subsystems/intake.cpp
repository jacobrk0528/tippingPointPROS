#include "main.h"
#include "globals.hpp"
#include "subsystems/intake.hpp"


void setIntake(int power) {
    intake.move_voltage(power);
}

void stopIntake() {
    intake.move_voltage(0);
    // CHECK IF THERE IS A STOP FUNCTION FOR MOTORS
}

void setStop(int type) {
    switch (type) {
        case 1:
            intake.set_brake_mode(pros::motor_brake_mode_e MOTOR_BRAKE_COAST);
        case 2:
            intake.set_brake_mode(pros::motor_brake_mode_e MOTOR_BRAKE_BRAKE);
        case 3:
            intake.set_brake_mode(pros::motor_brake_mode_e MOTOR_BRAKE_HOLD);
    }
}

void intakeControl() {
    `// set intakes based on control values
}

int runIntake(int power) {
    setIntake(power);
    pros::delay(20);
    return 1;
}
