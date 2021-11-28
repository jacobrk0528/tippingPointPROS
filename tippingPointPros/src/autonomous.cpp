#include "autonomous.hpp"

#include "subsystems/drivebase.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/lift.hpp"
#include "subsystems/pneumatics.hpp"
#include "subsystems/tilter.hpp"
#include "slew.hpp"

Claw claw;
Tilter tilter;
Intake intake;
DriveBase drivebase;
frontLift frontlift;

void Auton::reset() {
    std::cout << "reset is being called" << std::endl;
    drivebase.reset();
    std::cout << "past reset 1" << std::endl;
    tilter.reset();
    intake.reset();
    frontlift.reset();
}
void Auton::redRight() {
    reset();
    std::cout << "rightRed is being called" << std::endl;
    drivebase.move(50, .707, .681, .618);
    std::cout << "moving past move function" << std::endl;
}

void Auton::run() {
    std::cout << "auto is working" << std::endl;
    //reset();
    redRight();
}

void Auton::start(void* ignore) {
    Auton *that = static_cast<Auton*>(ignore);
    that -> run();
}
