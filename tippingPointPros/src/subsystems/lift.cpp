#include "main.h"
#include "globals.hpp"
#include "subsystems/lift.hpp"

class frontLift {
    private:

    public:
        frontLift();

        void reset() {
            frontLiftMotor.tare_position();
        }

        void setBreakType(int type) {
            switch (type) {
                case 1:
                    frontLiftMotor.set_brake_mode(pros::motor_brake_mode_e MOTOR_BRAKE_BRAKE);
                case 2:
                    frontLiftMotor.set_brake_mode(pros::motor_brake_mode_e MOTOR_BRAKE_COAST);
                default:
                    frontLiftMotor.set_brake_mode(pros::motor_brake_mode_e MOTOR_BRAKE_HOLD);
            }
        }
        
        void stop() {
            //frontLiftMotor. 
            // look up how to stop besides setting speed to 0
        }

        frontLift& withPID(double kp, double ki, double kd) {

        }

        frontLift& withSlew(int rate = 5){

        }

        frontLift& move(double target) {

        }
}