#include "main.h"
#include "globals.hpp"
#include "subsystems/intake.hpp"


Intake::Intake() {}
Intake::Intake() {
    reset();
}

void Intake::reset() {
    intake.move_velocity(0);
    intake.tare_position();
}


void Intake::setIntake(int power) {
    intake.move_voltage(power);
}

void Intake::stopIntake() {
    intake.move_voltage(0);
}

void Intake::setStop(int type) {
    switch (type) {
        case 1:
            intake.set_brake_mode(pros::motor_brake_mode_e MOTOR_BRAKE_COAST);
        case 2:
            intake.set_brake_mode(pros::motor_brake_mode_e MOTOR_BRAKE_BRAKE);
        case 3:
            intake.set_brake_mode(pros::motor_brake_mode_e MOTOR_BRAKE_HOLD);
    }
}

void Intake::intakeControl() {
    `// set intakes based on control values
}

int Intake::runIntake(){
    double joystickValue = Master.get_analog(ANALOG_LEFT_Y);
    setIntake(joystickValue);
    pros::delay(20);
    return 1;
}
