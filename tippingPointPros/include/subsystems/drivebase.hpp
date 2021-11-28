#include "globals.hpp"

#define RIGHT 1
#define LEFT 2
#define HOLD 1
#define COAST 2
#define CIRCUMFERENCE 8.635
#define CONVERSION 4169.079328314997

class DriveBase {
    public:

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
        DriveBase& withTurnSlew(int turnSlewRate = 5);

        /*
        set turn kP and kD values
        @param kP - kP value
        @param kD - kD value
        */
        DriveBase& withTurnPD(double kP, double kD);

        /*
        sets desired turn direction
        @param turnDriection - value either Right or Left
        */
        DriveBase& withTurnDirection(int turnDirection);

        /*
        sets drive slew rate
        @param slewRate - defult = 5
        */
        DriveBase& withSlew(int slewRate = 5);

        /*
        sets drive kP and kD values
        @param kP - kP value
        @param kD - kD value
        */
        DriveBase& withPD(double kP, double kD);

        /*
        sets angle for robot to stick to while driving and correction angle if it strays
        @param driveAngle - desired angle while driving
        @param correctionRate - the amount of correction applied, defult = 1
        */
        DriveBase& withHeading(double driveAngle, double correctionRate);

        /*
        sets justPD boolean 
        @param justPD_ - boolean used to determine if PD is to be run without slew
        */
        DriveBase& justPD(bool justPD_);

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
        DriveBase& turn(double desiredTurnAngle);

        /*
        Drives the robot a set distance

        Use withSlew & withPD with this.
        Slew rate control controls the speed of the turning until the speed of the slew rate and the PD loop meet then the PD loop takes control.

        @param target The wanted distance
        */
        DriveBase& drive(double target);

        /*
        drive until you reach certain distance.

        @param dist target distance.
        */
        DriveBase& driveDistAway(double dist);

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
        DriveBase& withSlop(double drive_tol_ = 10, double turn_tol_ = 1);

        /*
        Sets turn Variables used in Chassis::move();

        @param theta_ wanted angle.
        @param turn_kP_ kP constant.
        @param turn_kI_ kI constant.
        @param turn_kD_ kD constant.

        */
        DriveBase& withTurn(double theta_, double turn_kP_, double turn_kI_, double turn_kD_);

        /*
        Sets drive Variables.

        @param target wanted distance in inches.
        @param drive_kP_ kP constant.
        @param drive_kI_ kI constant.
        @param drive_kD_ kD constant.

        */
        DriveBase& move(double target, double drive_kP, double drive_kI, double drive_kD);

    private:
        static bool isSettled;

        static double IMUHeading;

        static double power, rate_drive, rate_turn;
        static double kP_drive, kI_drive, kD_drive, kP_turn, kD_turn;
        static double correction_rate;
        static int direction_turn;
        static double output;
        static int slew_a, slew_x;
        static int drive_theta;
        static bool justPID;
        static int tol;
        static double prevError;
        static int heading_diff;
        static bool oneSide;
        static double turnPrevError;

        static bool halt;
        static double turn_output;
        static double m_error, m_integral, m_derivative, m_prevError, m_power, LOutput, ROutput, turn_tol, drive_tol;
        static double t_error, t_integral, t_derivative, t_prevError, t_power, theta, turn_kP, turn_kI, turn_kD;


};