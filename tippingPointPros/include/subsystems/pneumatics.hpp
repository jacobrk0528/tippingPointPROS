#include "globals.hpp"

#define OPEN 1
#define CLOSE 2

class Claw {
    public: 
        //FRONT PNEUMATICS
        // grab something
        void closeFront();
        // release
        void openFront();

        //DRIVER CONTROL
        void frontClaw();

        static void start(void* ignore);
    private:
        static int state;
};

