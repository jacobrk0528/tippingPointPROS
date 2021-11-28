#include "pneumatics.hpp"

int Claw::state = OPEN;

//FRONT PNEUMATICS
// grab something
void Claw::closeFront() {
    frontClawPiston.set_value(true);
    state = CLOSE;
}
// release
void Claw::openFront(){
    frontClawPiston.set_value(false);
    state = OPEN;
}

//DRIVER CONTROL
void Claw::frontClaw(){
    while(true){
        if(Master.get_digital(DIGITAL_X)) {
            if(state == OPEN) {
                closeFront();
                state = CLOSE;
            } else {
                openFront();
                state = OPEN;
            }
        }
    }
}

void Claw::start(void* ignore) {
    Claw *that = static_cast<Claw*>(ignore);
    that -> frontClaw();
}
