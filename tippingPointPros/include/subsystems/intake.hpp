#include "globals.hpp"

class Intake {
    public:
        Intake();
        void reset();
        void setIntake(int power);
        void stopIntake();
        void setStop(int type);

        void runIntake();

        static void start(void* ignore);
    private:
        static double joystickValue;

};
