#include "pneumatics.hpp"

int state = OPEN;

Claw::Claw() {}

//FRONT PNEUMATICS
// grab something
void Claw::closeFront() {
    forntClawPiston.set_value(true);
}
// release
void Claw::openFront(){
    forntClawPiston.set_value(false);
}


//BACK PNEUMATICS
//grab something
void Claw::closeBack(){
    backClawPiston.set_value(true);
}
//release 
void Claw::openBack(){
    backClawPiston.set_value(false);
}

//DRIVER CONTROL
void Claw::frontClaw(){
    while(true){
        if(Master.get_digital(DIGITAL_X)) {
            if(state == OPEN) {
                closeFront();
            } else {
                openFront();
            }
        }
    }
}

void Claw::start(void* ignore) {
    Claw *that = static_cast<Claw*>(ignore);
    that -> frontClaw();
}

int Claw::backClaw(){
    if(Master.get_digital(DIGITAL_B)) {
        if(state == OPEN) {
            closeBack();
        } else {
            openBack();
        }
    }
}