#include "autonomous.hpp"

#include "subsystems/drivebase.hpp"
#include "subsystems/lift.hpp"
#include "subsystems/pneumatics.hpp"
#include "slew.hpp"

Claw claw;
DriveBase drivebase;
frontLift frontlift;
backLift backlift;

void Auton::reset() {
    drivebase.reset();
    frontlift.reset();
}
void Auton::redAWP() {
    reset();
    backlift.startMove(-100);
    pros::delay(1300);
    backlift.stop();
    drivebase.withSlew(10).withPD(.06, .045).justPD(false).withDirection(REVERSE).drive(500);
    pros::delay(100);
    backlift.startMove(100);
    pros::delay(1000);
    backlift.stop();
    
    drivebase.withSlew(3).withPD(.2, .14).withDirection(FORWARD).drive(600); // consider making a drive curve function
    // turn 90 degrees right
    pros::delay(2000);
    drivebase.withSlew(200).withPD(.4, .34).withDirection(FORWARD).drive(800);
    // turn 90 degrees right
    pros::delay(2000);
    backlift.startMove(-25);
    drivebase.withFancySlew(200).withPD(.26, .21).withDirection(FORWARD).drive(2000);
    backlift.stop();
    claw.closeFront();
    drivebase.withFancySlew(200).withPD(.26, .21).withDirection(REVERSE).drive(10);
    claw.openFront();
    // turn to face middle mobil goal
    drivebase.withFancySlew(200).withPD(.26, .21).withDirection(REVERSE).drive(35);
    backlift.startMove(100);
    drivebase.withFancySlew(200).withPD(.26, .21).withDirection(FORWARD).drive(40);
    backlift.stop();
}

void Auton::run() {
    //reset();
    redAWP();
}

void Auton::start(void* ignore) {
    Auton *that = static_cast<Auton*>(ignore);
    that -> run();
}
