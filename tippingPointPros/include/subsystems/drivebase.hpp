#pragma once
#include "globals.hpp"
#include "main.h"

class driveBase {
    public:
        driveBase();

        //reset drive base encoders and sensors
        void reset();

        //reset odometry sensors
        void odomReset();

        /*
        set break status
        @param int - brakeState state of brake (hold, coast, break)
        */
        void setBreak(int breakState);

        //stops drive
        void stop();

        /*
        set turn slew rate
        @param turnSlewRate - Default = 5
        */
        driveBase& withTurnSlew(int turnSlewRate = 5);

        /*
        set turn kP and kD values
        @param kP - kP value
        @param kD - kD value
        */
        driveBase& withTurnPD(double kP, double kD);

        /*
        sets desired turn direction
        @param turnDriection - value either Right or Left
        */
        driveBase& withTurnDirection(int turnDirection);

        /*
        sets drive slew rate
        @param slewRate - defult = 5
        */
        driveBase& withSlew(int slewRate = 5);

        /*
        sets drive kP and kD values
        @param kP - kP value
        @param kD - kD value
        */
        driveBase& withPD(double kP, double kD);

        /*
        sets angle for robot to stick to while driving and correction angle if it strays
        @param driveAngle - desired angle while driving
        @param correctionRate - the amount of correction applied, defult = 1
        */
        driveBase& withHeading(double driveAngle, double correctionRate);

        /*
        sets justPD boolean 
        @param justPD_ - boolean used to determine if PD is to be run without slew
        */
        driveBase& justPD(bool justPD_);

        /*
        calcuate weather to turn left or right 
        @param currentPos - current heading
        @param targetPos - target heading
        */
        void calcTurnDirection(int currentPos, int targetPos);

        /*
        turn robot to set angle
        
        use withTurnSlew and withturnPD with this
        Slew rate control controls the speed of the turning until the speed of the slew rate and the PD loop meet then the PD loop takes control.
        @param desiredTurnAngle
        */
        driveBase& turn(double desiredTurnAngle);

        /*
        Drives the robot a set distance

        Use withSlew & withPD with this.
        Slew rate control controls the speed of the turning until the speed of the slew rate and the PD loop meet then the PD loop takes control.

        @param target The wanted distance
        */
        driveBase& drive(double target);

        /*
        drive until you reach certain distance.

        @param dist target distance.
        */
        driveBase& driveDistAway(double dist);

        /*
        Sets halt boolean. Controls whether motors stop moving after reaching tolerance.

        @param halt_ 1 to stop motors.
        */
        void waitUntilSettled(bool halt_ = 1);

        /*
        Set slop value for drive and turn. Use with move();

        @param drive_tol lateral mvmt slop.
        @param turn_tol turn mvmt slop.
        */
        Chassis& withSlop(double drive_tol_ = 10, double turn_tol_ = 1);

        /*
        Sets turn Variables used in Chassis::move();

        @param theta_ wanted angle.
        @param turn_kP_ kP constant.
        @param turn_kI_ kI constant.
        @param turn_kD_ kD constant.

        */
        Chassis& withTurn(double theta_, double turn_kP_, double turn_kI_, double turn_kD_);

        /*
        Sets drive Variables.

        @param target wanted distance in inches.
        @param drive_kP_ kP constant.
        @param drive_kI_ kI constant.
        @param drive_kD_ kD constant.

        */
        Chassis& move(double target, double drive_kP, double drive_kI, double drive_kD);
}