#include "main.h"
#include "tilter.hpp"
#include "globals.hpp"

int Tilter::slewRate = 5;
int Tilter::currentPos = tilter.get_position();
int Tilter::acceptableError = 5;
int Tilter::outputPower = 0;
bool Tilter::slew = true;

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
}

Tilter& Tilter::move(int target){
    while ((fabs(target) > (fabs(currentPos)+fabs(acceptableError))) || (fabs(target) < (fabs(currentPos)-fabs(acceptableError)))) {
        while (outputPower < 75) {
            if(slew){
                outputPower += slewRate;
            } else {
                if(target > currentPos) {
                    outputPower = 127;
                } else {
                    outputPower = -127;
                }
            }
        }
    }
}

int Tilter::runTilter() {
    if(Master.get_digital(DIGITAL_Y)) {
        Tilter::move(RING).withSlew();
    } else if (Master.get_digital(DIGITAL_A)) {
        Tilter::move(RESTING).withSlew();
    }
}
