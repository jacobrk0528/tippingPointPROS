#include "main.h"
#include "globals.hpp"

#define RING //encoder number for pos to pick up rings
#define RESTING 0 // encoder number for pos to rest -- most likly 0

class Tilter {
    public:
        Tilter();

        void reset();

        int runTilter();

        Tilter& withSlew(int rate = 5);

        Tilter& move(int target);

    private:
        int slewRate;
        int currentPos;
        int acceptableError;
        int outputPower;
        bool slew;
}