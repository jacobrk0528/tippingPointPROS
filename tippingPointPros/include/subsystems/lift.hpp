#include "globals.hpp"

#define UP 1
#define DOWN -1
#define NEITHER 0

class frontLift {
    private:
        static double groundValue;
        static double kP, kI, kD;
        static double error, prevError, derivitive, intagral;
        static double output, power;
        static int tol;
        static double slewValue;
        static double currentPos;
        static bool justPID_;
        static int dir;
    public:
        
        void reset();

        void setBreakType(int type);

        void stop();

        void justPID();

        frontLift& withPID(double kp, double ki, double kd);

        frontLift& withSlew(int rate = 5);

        frontLift& move(double target);

        static void start(void* ignore);

        void driver();
};

class backLift{
    private:
        static double groundValue;
        static double kP, kI, kD;
        static double error, prevError, derivitive, intagral;
        static double output, power;
        static int tol;
        static double slewValue;
        static double currentPos;
        static bool justPID_;
        static int dir;
    public:
        
        void reset();

        void setBreakType(int type);

        void stop();

        void justPID();

        backLift& withPID(double kp, double ki, double kd);

        backLift& withSlew(int rate = 5);

        backLift& move(double target);

        static void start(void* ignore);
};