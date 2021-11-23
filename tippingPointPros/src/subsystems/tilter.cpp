#include "main.h"
#include "tilter.hpp"
#include "globals.hpp"

int Tilter::slewRate = 5;
int Tilter::currentPos = tilter.get_position();
int Tilter::acceptableError = 5;
int Tilter::outputPower = 0;
int Tilter::dir = 0;

double Tilter::kP, Tilter::kD;
double Tilter::prevError = 0;
double Tilter::error, Tilter::derivitive;

bool Tilter::slew = false;
bool Tilter::PD = false;

int Tilter::controlType = MOTOR;


Tilter::Tilter();
Tilter::Tilter() {
    reset();
}

Tilter::reset() {
    tilter.move_velocity(0);
    tilter.tare_position();
}

Tilter& Tilter::withSlew(int rate = 5) {
    Tilter::slewRate = rate;
    Tilter::controlType = SLEW;
    return *this;
}
Tilter& Tilter::withPD(double kP_, double kD_) {
    Tilter::kP = kP_;
    Tilter::kD = kD_;
    Tilter::controlType = PD;
    return *this;
}

Tilter& Tilter::move(int target){
    while ((fabs(target) > (fabs(currentPos)+fabs(acceptableError))) || (fabs(target) < (fabs(currentPos)-fabs(acceptableError)))) {
        dir = target/fabs(target);
        while (outputPower < MAXOUTPUT) {
            switch(controlType) {
                case SLEW: {
                    outputPower += slewRate;
                    tilter.move_velocity(outputPower*dir);
                    break;
                }
                case PD: {
                    error = fabs(target - currentPos);
                    derivitive = error-prevError;
                    prevError = error;
                    outputPower = kP*error + kD*derivitive;
                    tilter.move_voltage(outputPower*dir);
                    break;
                }
                case MOTOR: {
                    outputPower = MAXOUTPUT+1;
                    tilter.move_velocity(outputPower*dir);
                    break;
                }
            }
            if (outputPower >= MAXOUTPUT) {
                break;
            }
        }
    }
    tilter.move_voltage(0);
}

int Tilter::runTilter() {
    if(Master.get_digital(DIGITAL_Y)) {
        Tilter::move(RING).withSlew();
    } else if (Master.get_digital(DIGITAL_A)) {
        Tilter::move(RESTING).withPD(.00823, .00698);
    }
}
