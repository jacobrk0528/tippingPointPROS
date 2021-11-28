#include "globals.hpp"

class Intake {
    public:
        Intake();
        void reset();
        void setIntake(int power);
        void stopIntake();
        void setStop(int type);
        void intakeControl();

        int runIntake(int power);
    private:
        static double joystickValue;

};
