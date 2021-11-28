#include "globals.hpp"

#define RIGHT 1
#define LEFT -1 // adjust based on which is left and which is right (pos or neg)

#define FORWARD 1
#define REVERSE -1

#define HOLD 1
#define COAST 2
#define BREAK 3

class DriveBase {
    public:

        //reset drive base encoders and sensors
        void reset();

        /*
        set break status
        @param int - brakeState state of brake (hold, coast, break)
        */
        void setBreak(int breakState = COAST);

        //stops drive
        void stop();

        // gets average position of the 4 drive motors
        double getAvgPos();

        /*
        set turn slew rate
        @param turnSlewRate - Default = 5
        */
        DriveBase& withTurnSlew(int turnSlewRate_ = 5);

        /*
        set turn kP and kD values
        @param kP - kP value
        @param kD - kD value
        */
        DriveBase& withTurnPD(double KP, double KD);

        /*
        sets desired turn direction
        @param turnDriection - value either Right or Left
        */
        DriveBase& withTurnDirection(int turnDirection_);

        /*
        sets drive slew rate
        @param slewRate - defult = 5
        */
        DriveBase& withSlew(int slewRate_ = 5);

        /*
        sets drive kP and kD values
        @param kP - kP value
        @param kD - kD value
        */
        DriveBase& withPD(double KP, double KD);

        /*
        sets justPD boolean 
        @param justPD_ - boolean used to determine if PD is to be run without slew
        */
        DriveBase& justPD(bool justpd);

        /* 
        sets desired drive direction
        @param driveDirection_ - value either forward or reverse
        */
        DriveBase& withDirection(int driveDirection_);

        /*
        turn robot to set angle - target in degrees
        
        use withTurnSlew and withturnPD with this
        Slew rate control controls the speed of the turning until the speed of the slew rate and the PD loop meet then the PD loop takes control.
        @param desiredTurnAngle
        */
        void turn(double target);

        /*
        Drives the robot a set distance

        Use withSlew & withPD with this.
        Slew rate control controls the speed of the turning until the speed of the slew rate and the PD loop meet then the PD loop takes control.

        @param target The wanted distance
        */
        void drive(double target);


    private:
        static double turnSlewRate;
        static double turnKP, turnKD;
        static double currentHeading;
        static double turnDirection;

        static double slewRate;
        static double kp, kd;
        static double driveDirection;

        static double currentPos;

        static int acceptableError;

        static bool justPD_;
};