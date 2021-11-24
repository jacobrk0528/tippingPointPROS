#include "main.h"

#define RING //encoder number for pos to pick up rings --> must be positive
#define RESTING 0 // encoder number for pos to rest --> must be negitive 
// i could have it reset after each movement. meaning ring and resting would be the same value just one negitive and one positive

#define SLEW 1
#define PD 2
#define MOTOR 3
#define MAXOUTPUT 75

class Tilter {
    public:
        Tilter();

        void reset();

        int runTilter();

        Tilter& withSlew(int rate = 5);

        Tilter& move(int target);

    private:
        static int slewRate;
        static int currentPos;
        static int acceptableError;
        static int outputPower;
        static int dir;
        static double error, prevError, derivitive; 
        static double kP, kD;
        static bool slew;
        static bool PD;

        static int controlType;
}