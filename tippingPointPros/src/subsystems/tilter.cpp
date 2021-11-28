#include "tilter.hpp"

int Tilter::slewRate = 5;
int Tilter::currentPos = tilterMotor.get_position();
int Tilter::acceptableError = 0;
int Tilter::outputPower = 0;
int Tilter::dir = 0;

double Tilter::kP = .09, Tilter::kD = .081;
double Tilter::prevError = 0;
double Tilter::error, Tilter::derivitive;

bool Tilter::slew = false;
bool Tilter::PD_ = false;

int Tilter::controlType = MOTOR;

Tilter::Tilter() {
    reset();
}

void Tilter::reset() {
    tilterMotor.move_velocity(0);
    tilterMotor.tare_position();
}

double Tilter::getValue() {
    return tilterMotor.get_position();
}

Tilter& Tilter::withSlew(int rate) {
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

void Tilter::move(double target){
    std::cout << "text2" << std::endl;
    while ((fabs(target) > (fabs(currentPos)+fabs(acceptableError))) || (fabs(target) < (fabs(currentPos)-fabs(acceptableError)))) {
        dir = target/fabs(target);
        while (outputPower < MAXOUTPUT) {
            currentPos = tilterMotor.get_position();
            switch(controlType) {
                case SLEW: {
                    outputPower += slewRate;
                    tilterMotor.move_velocity(outputPower*dir);
                    break;
                }
                case PD: {
                    error = fabs(target - currentPos);
                    derivitive = error-prevError;
                    prevError = error;
                    outputPower = kP*error + kD*derivitive;
                    tilterMotor.move_voltage(outputPower*dir);
                    break;
                }
                case MOTOR: {
                    outputPower = MAXOUTPUT+1;
                    tilterMotor.move_velocity(outputPower*dir);
                    break;
                }
            }
            if (outputPower >= MAXOUTPUT) {
                break;
            }
        }
    }
    tilterMotor.move_voltage(0);
}

void Tilter::runTilter() {
    while (true) {
        if(Master.get_digital(DIGITAL_Y)) {
            //Tilter::move(RING);
            tilterMotor.move_voltage(6000);
        } else if (Master.get_digital(DIGITAL_A)) {
            //Tilter::move(RESTING);
            tilterMotor.move_voltage(-6000);
        } else {
            tilterMotor.move_voltage(0);
        }
    }
}

void Tilter::start(void* ignore) {
    Tilter *that = static_cast<Tilter*>(ignore);
    that -> runTilter();
    
}
