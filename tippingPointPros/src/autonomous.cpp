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
    drivebase.reset();
    tilter.reset();
    intake.reset();
    frontlift.reset();
}
void Auton::redRight() {
    reset();
    drivebase.withSlew(5).withPD(.2, .1).justPD(true).withDirection(FORWARD).drive(800);
}

void Auton::run() {
    //reset();
    redRight();
}

void Auton::start(void* ignore) {
    Auton *that = static_cast<Auton*>(ignore);
    that -> run();
}
