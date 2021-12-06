#include "globals.hpp"

class Auton {
    public:
        void reset();
        void redAWP();
        void redRight();
        void redLeft();
        void blueRight();
        void blueLeft();

        void run();

        void runSkills();

        static void start(void* ignore);
};