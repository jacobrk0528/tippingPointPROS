#include "main.h"
#include "globals.hpp"
#include "subsystems/pneumatics.hpp"

int state = OPEN;

Claw::Claw() {}

//FRONT PNEUMATICS
// grab something
void Claw::closeFront() {
    forntClaw.set_value(true);
}
// release
void Claw::openFront(){
    forntClaw.set_value(false);
}


//BACK PNEUMATICS
//grab something
void Claw::closeBack(){
    backClaw.set_value(true);
}
//release 
void Claw::openBack(){
    backClaw.set_value(false);
}

//DRIVER CONTROL
int Claw::frontClaw(){
    if(Master.get_digital(DIGITAL_X)) {
        if(state == OPEN) {
            closeFront();
        } else {
            openFront();
        }
    }
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