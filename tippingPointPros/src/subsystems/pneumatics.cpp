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
        if(pros::competition::is_autonomous() == 0) {
            if(Master.get_digital(DIGITAL_X)) {
                closeFront();
                state = CLOSE;
            } else if (Master.get_digital(DIGITAL_B)) {
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
