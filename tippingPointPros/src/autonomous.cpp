#include "autonomous.hpp"
/*
static DriveBase drivebase;
static Intake intake;
static frontLift frontlift;
static backLift backlift;
static Claw claw;
static Tilter tilter;

void reset() {
    drivebase.reset()
    intake.reset()
    frontlift.reset()
    backlift.reset();
    tilter.reset();
}
void redRight() {
    tilter.move(RESTING);
    drivebase.withSlew(5).withPD().move(600, .0089, .0032, .0063);
    drivebase.waitUntilSettled();
    claw.closeFront();
    tilter.move(RING);
    drivebase.withTurnDirection(LEFT).turn(90); // left
    drivebase.move(300, .0089, .0032, .0063);
    drivebase.withTurnDirection(LEFT).turn(90);
    drivebase.waitUntilSettled();
    intake.runIntake(127);
    drivebase.withPD(.0089, .0063).withSlew(5).move(-600, .0089, .0032, .0063)
}

Auton& run() {
    reset();
    redRight();
}*/
