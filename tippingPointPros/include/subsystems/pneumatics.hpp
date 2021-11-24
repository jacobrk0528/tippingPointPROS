#include "main.h"

#define OPEN 1
#define CLOSE 2

class Claw {
    public: 
        Claw();
        //FRONT PNEUMATICS
        // grab something
        void closeFront();
        // release
        void openFront();


        //BACK PNEUMATICS
        //grab something
        void closeBack();
        //release 
        void openBack();

        //DRIVER CONTROL
        int frontClaw();
        int backClaw();
    private:
        int state;
}

