#include "main"

class Auton {
    public:
        void reset();
        void redRight();
        void redLeft();
        void blueRight();
        void blueLeft();

        Auton& run();

        Auton& runSkills();
};