#include "drivebase.hpp"

double DriveBase::turnSlewRate;
double DriveBase::turnKP, DriveBase::turnKD;
double DriveBase::currentHeading = 0;
double DriveBase::turnDirection;

double DriveBase::slewRate;
double DriveBase::slew_a = 200;
double DriveBase::slew_x;
double DriveBase::kp, DriveBase::kd;
double DriveBase::driveDirection;

double DriveBase::currentPos;


double DriveBase::acceptableError = 5;

bool DriveBase::justPD_ = false;
bool DriveBase::normalSlew = false;

void DriveBase::reset() {
    leftFrontMotor.move_velocity(0);
    leftBackMotor.move_velocity(0);
    rightFrontMotor.move_velocity(0);
    rightBackMotor.move_velocity(0);

    leftFrontMotor.tare_position();
    leftBackMotor.tare_position();
    rightFrontMotor.tare_position();
    rightBackMotor.tare_position();
}

void DriveBase::setBreak(int breakState){
    switch (breakState) {
        case HOLD: {
            leftFrontMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
            leftBackMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
            rightFrontMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
            rightBackMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
            break;
        };
        case COAST: {
            leftFrontMotor.set_brake_mode(MOTOR_BRAKE_COAST);
            leftBackMotor.set_brake_mode(MOTOR_BRAKE_COAST);
            rightFrontMotor.set_brake_mode(MOTOR_BRAKE_COAST);
            rightBackMotor.set_brake_mode(MOTOR_BRAKE_COAST);
            break;
        };
        case BREAK: {
            leftFrontMotor.set_brake_mode(MOTOR_BRAKE_BRAKE);
            leftBackMotor.set_brake_mode(MOTOR_BRAKE_BRAKE);
            rightFrontMotor.set_brake_mode(MOTOR_BRAKE_BRAKE);
            rightBackMotor.set_brake_mode(MOTOR_BRAKE_BRAKE);
            break;
        };
    }
}

void DriveBase::stop() {
    leftFrontMotor.move_velocity(0);
    leftBackMotor.move_velocity(0);
    rightFrontMotor.move_velocity(0);
    rightBackMotor.move_velocity(0);
}

double DriveBase::getAvgPos(){
    double leftPos = (fabs(leftFrontMotor.get_position()) + fabs(leftBackMotor.get_position()))/2;
    double rightPos = (fabs(rightFrontMotor.get_position()) + fabs(rightBackMotor.get_position()))/2;
    return (leftPos + rightPos)/2;
}

DriveBase& DriveBase::withTurnSlew(int turnSlewRate_) {
    turnSlewRate = turnSlewRate_;
    return *this;
}

DriveBase& DriveBase::withTurnPD(double KP, double KD){
   turnKP = KP;
   turnKD = KD;
   return *this; 
}

DriveBase& DriveBase::withTurnDirection(int turnDirection_){
    turnDirection = turnDirection_;
    return *this;
}

DriveBase& DriveBase::withSlew(int slewRate_){
    slewRate = slewRate_;
    normalSlew = true;
    return *this;
}

DriveBase& DriveBase::withFancySlew(int slewa){
    slew_a = slewa;
    normalSlew = false;
    return *this;
}

DriveBase& DriveBase::withPD(double KP, double KD){
    kp = KP;
    kd = KD;
    return *this;
}

DriveBase& DriveBase::justPD(bool justpd){
    justPD_ = justpd;
    return *this;
}

DriveBase& DriveBase::withDirection(int driveDirection_){
    driveDirection = driveDirection_;
    return *this;
}

void DriveBase::turn(double target) {
    double error;
    double prevError = 0;
    double derivitive;
    double output = 0;

    currentHeading = inertial.get_rotation();

    while(fabs(currentHeading) < (target - acceptableError) || fabs(currentHeading) > target + acceptableError) {
        // PD Loop
        currentHeading = inertial.get_rotation();

        error = currentHeading - target;
        derivitive = error - prevError;

        double power = error*kp + derivitive*kd;

        // Slew
        if (output < power && !justPD_) {
            output += slewRate;
        } else {
            output = power;
        }

        // set motor power
        leftFrontMotor.move_voltage(output * turnDirection);
        leftBackMotor.move_voltage(output * turnDirection);
        rightFrontMotor.move_voltage(output * turnDirection);
        rightBackMotor.move_voltage(output * turnDirection);

        prevError = error;
        pros::delay(10);
    }
}

void DriveBase::drive(double target) {
    reset();
    double error;
    double prevError = 0;
    double derivitive;
    double output = 0;

    int slewOutput = 0;

    currentPos = getAvgPos();

    while(currentPos < (target - acceptableError) || currentPos > (target + acceptableError)) {
        //PD loop
        currentPos = getAvgPos();

        error = target - currentPos;
        derivitive = error - prevError;

        double power = error*kp + derivitive*kd;

        if (normalSlew) {
            // simple slew function
            if (output < power && !justPD_) {
                output += slewRate;
            } else {
                output = power * CONSTANT;
            }
        } else {
            //fancy slew function
            if(output < power && !justPD_) {
                if(target>0) {
                    output = fabs(((-2*slew_a)/pow(M_E, (2.2*slew_x)/slew_a)+1)+slew_a);
                } else { 
                    output = ((-2*slew_a)/pow(M_E, (2.2*slew_x)/slew_a)+1)+slew_a;
                }
            } else {
                output = power;
            }

            slew_x += .001;
        }

        // set motor power

        leftFrontMotor.move_velocity(output * driveDirection);
        leftBackMotor.move_velocity(output * driveDirection);
        rightFrontMotor.move_velocity(output * driveDirection);
        rightBackMotor.move_velocity(output * driveDirection);

        prevError = error;
        pros::delay(2);
    }
    stop();
}