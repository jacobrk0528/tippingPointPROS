#include "main.h"

class Slew {
  public:
    /*
    Starts Drive Tank Control with the given slew values.

    @param fwdAccel Forward acceleration rate.
    @param deccel Decceleration rate.
    @param revAccel Reverse/backward accerleration rate.
    */
    int tankDrive(double fwdAccel, double deccel, double revAccel);
    int arcadeDrive(double fwdAccel, double deccel, double revAccel);
    int frontLift(double fwdAccel, double deccel, double revAccel);
    int backLift(double fwdAccel, double deccel, double revAccel);
    


  private:
    static int leftSlewOutput, rightSlewOutput;
    static int xSlewOutput, ySlewOutput;
    static int driveMax;
    static int leftJoystick, rightJoystick;
    static int leftSide, rightSide;
    static int rightTarget, leftTarget;

    int xSlewOutput, ySlewOutput;

    int frontLiftOutput;
    int frontLiftMax;
    int frontLiftTarget;
    int frontLiftUpButton, frontLiftdownButton;

    int backLiftOutput;
    int backLiftMax;
    int backLiftTarget;
    int backLiftUpButton, backLiftdownButton;
};
