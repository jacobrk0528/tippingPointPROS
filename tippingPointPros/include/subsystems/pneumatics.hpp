#include "globals.hpp"

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
        void frontClaw();
        void backClaw();

        static void start(void* ignore);
    private:
        int state;
};

