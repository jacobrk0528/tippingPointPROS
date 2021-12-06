#include "globals.hpp"

#define UP -1
#define DOWN 1
#define NEITHER 0

class frontLift {
    private:
        static double kP, kI, kD;
        static double acceptableError;
        static double slewValue;
        static double currentPos;
        static bool justPID_;
        static int dir;

    public:
        void reset();

        void setBreakType(int type);

        void stop();

        double getAvgPos();

        frontLift& withDirection(int direction);

        frontLift& justPID();

        frontLift& withPID(double kp, double ki, double kd);

        frontLift& withSlew(int rate = 5);

        void move(double target);

        void startMove(int speed);

        static void start(void* ignore);

        void driver();
};

class backLift{
    private:
        static double kP, kI, kD;
        static double acceptableError;
        static double slewValue;
        static double currentPos;
        static bool justPID_;
        static int dir;

    public:
        void reset();

        void setBreakType(int type);

        void stop();

        double getAvgPos();

        backLift& withDirection(int direction);

        backLift& justPID();

        backLift& withPID(double kp, double ki, double kd);

        backLift& withSlew(int rate = 5);

        void move(double target);

        void startMove(int speed);

        static void start(void* ignore);

        void driver();
};