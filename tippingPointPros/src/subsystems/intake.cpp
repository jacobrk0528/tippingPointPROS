#include "main.h"
#include "globals.hpp"
#include "subsystems/intake.hpp"

double Intake::joystickValue = 0;

static Tilter tilter;

Intake::Intake() {}
Intake::Intake() {
    reset();
}

void Intake::reset() {
    intake.move_velocity(0);
    intake.tare_position();
}


void Intake::setIntake(int power) {
    intakeMotor.move_voltage(power);
}

void Intake::stopIntake() {
    intakeMotor.move_voltage(0);
}

void Intake::setStop(int type) {
    switch (type) {
        case 1:
            intakeMotor.set_brake_mode(pros::motor_brake_mode_e MOTOR_BRAKE_COAST);
        case 2:
            intakeMotor.set_brake_mode(pros::motor_brake_mode_e MOTOR_BRAKE_BRAKE);
        case 3:
            intakeMotor.set_brake_mode(pros::motor_brake_mode_e MOTOR_BRAKE_HOLD);
    }
}

void Intake::intakeControl() {
    `// set intakes based on control values
}

int Intake::runIntake(){
    if(tilter.getValue() > 20) {
        joystickValue = 127;
    }
    
    joystickValue = Master.get_analog(ANALOG_LEFT_Y);
    joystickValue *= 90;  // joystick returns up to 127 --- voltage takes values up to 12000
    setIntake(joystickValue);
    pros::delay(20);
    return 1;
}
