#include "main.h"
#include "globals.hpp"

class frontLift {
    private:
        static groundValue = 0;
    public:
        frontLift();
        
        void reset();

        void setBreakType(int type);

        void stop();

        frontLift& withPID(double kp, double ki, double kd);

        frontLift& withslew(int rate = 5);

        frontLift& move(double target);
};

class backLift{
    private:
        static groundValue = 0;
    public:
        backLift();

        void reset();

        void setBreakType();

        void stop();

        backLift& withPID(double kp, double, ki, double kd);

        backLift& withSlew(int rate = 5);

        backLift& move();
}