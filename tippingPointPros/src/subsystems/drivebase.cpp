#include "main.h"
#include "subsystems/drivebase.hpp"
#include "globals.hpp"

bool driveBase::isSettled = true;
bool driveBase::justPID = false;
bool driveBase::oneSide;
bool driveBase::halt;

double driveBase::IMUHeading = inertial.get_heading();
double driveBase::power;
double driveBase::kP_drive, driveBase::kD_drive, driveBase::kP_turn, driveBase::kD_turn;
double driveBase::rate_drive, driveBase::rate_turn, driveBase::correction_rate;
double driveBase::output = 1;
double driveBase::prevError = 0;
double driveBase::turnPrevError = 0;
double driveBase::m_error, driveBase::m_integral, driveBase::m_derivative, driveBase::m_prevError, driveBase::m_power, driveBase::LOutput, driveBase::ROutput, driveBase::drive_tol = 10, driveBase::turn_tol = 1, driveBase::t_error, driveBase::t_integral, driveBase::t_derivative, driveBase::t_prevError, driveBase::theta, driveBase::turn_kP, driveBase::turn_kI, driveBase::turn_kD, driveBase::turn_output;

int driveBase::drive_theta;
int driveBase::direction_turn;
int driveBase::slew_a = 600, driveBase::slew_x = 1;
int driveBase::tol, driveBase::heading_diff;


driveBase::driveBase() {}
driveBase::~driveBase() {
    reset();
}

void driveBase::reset() {
    rightFrontMotor.move_velocity(0);
    rightBackMotor.move_velocity(0);
    leftFrontMotor.move_velocity(0);
    leftBackMotor.move_velocity(0);
    rightFrontMotor.tare_position();
    rightBackMotor.tare_position();
    leftFrontMotor.tare_position();
    leftBackMotor.tare_position();

    inertial.reset();

    LOdometer.reset();
    ROdometer.reset();
    ROdometer.set_reversed(1);

    while(inertial.is_calibrating()) {
        pros::delay(5);
    }
}

void driveBase::odomReset(){
    LOdometer.reset_position();
    ROdometer.reset_position();
}

void driveBase::setBreak(int breakState){
    switch (breakState) {
        case 1: {
            rightFrontMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
            rightBackMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
            leftFrontMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
            leftBackMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
            break;}
        case 2: {
            rightFrontMotor.set_brake_mode(MOTOR_BRAKE_BRAKE);
            rightBackMotor.set_brake_mode(MOTOR_BRAKE_BRAKE);
            leftFrontMotor.set_brake_mode(MOTOR_BRAKE_BRAKE);
            leftBackMotor.set_brake_mode(MOTOR_BRAKE_BRAKE);
            break;}
        default: {
            rightFrontMotor.set_brake_mode(MOTOR_BRAKE_COAST);
            rightBackMotor.set_brake_mode(MOTOR_BRAKE_COAST);
            leftFrontMotor.set_brake_mode(MOTOR_BRAKE_COAST);
            leftBackMotor.set_brake_mode(MOTOR_BRAKE_COAST);
            break;}
    }
}

void driveBase::stop() {
    rightFrontMotor.move(0);
    rightBackMotor.move(0);
    leftFrontMotor.move(0);
    leftBackMotor.move(0);

    odomReset();

    justPD = false;
}

driveBase& driveBase::withTurnSlew(int turnSlewRate = 5) {
    rate_turn = turnSlewRate;
    return *this;
}

driveBase& driveBase::withTurnPD(double kP_, double kD_) {
    kP_turn = kP_;
    kD_turn = kD_;
    return *this;
}

driveBase& driveBase::withTurnDirection(int turnDirection) {
     direction_turn = turnDirection;
     return *this;
}

driveBase& driveBase::withSlew(int slewRate = 5) {
    slew_a = slewRate;
}

driveBase& driveBase::withPD(double kP_, double kD_) {
    kP_drive = kP_;
    kD_drive = kD_;
    return *this;
}

driveBase& driveBase::withHeading(double driveAngle, double correctionRate) {
    drive_theta = driveAngle;
    correction_rate = correctionRate;
    return *this;
}

driveBase& driveBase::justPD(bool justPD_) {
    justPID = justPD_;
    return *this;
}

driveBase& driveBase::calcTurnDirection(int currentPos, int targetPos) {
    if(currentPos == 0) {
        if(abs(heading_diff) >= 180) {
            direction_turn = LEFT;
        }
        else {
            direction_turn = RIGHT;
        }
    }
    if(heading_diff < 0) {
        if(abs(heading_diff) >= 180) {
            direction_turn = LEFT;
        }
        else {
            direction_turn = RIGHT;
        }
    }
    if(heading_diff > 0) {
        if(abs(heading_diff) >= 180) {
            direction_turn = RIGHT;
        }
        else {
            direction_turn = LEFT;
        }
    }
}

driveBase& driveBase::turn(double desiredTurnAngle) {
  isSettled = false;
  while(IMUHeading != desiredTurnAngle) {
    double theta = desiredTurnAngle;
    IMUHeading = inertial.get_heading();
    if(IMUHeading > 355) {
        if(direction_turn ==LEFT) {
            IMUHeading=360;
        }
        else {
            IMUHeading = 0;
        }
    }

    calcDir(IMUHeading, desiredTurnAngle);

    double error = fabs(theta - IMUHeading);
    double derivative = error - prevError;
    double prevError = error;
    power = error*kP_turn + derivative*kD_turn;

    if(output < power && !justPID) {
        output += rate_turn;
    } else {
        output = power;
    }

    switch (direction_turn){
      case LEFT: {
        rightFrontMotor.move_velocity(output);
        rightBackMotor.move_velocity(output);
        leftFrontMotor.move_velocity(-output);
        leftBackMotor.move_velocity(-output);
        break;}
      case RIGHT: {
        rightFrontMotor.move_velocity(-output);
        rightBackMotor.move_velocity(-output);
        leftFrontMotor.move_velocity(output);
        leftBackMotor.move_velocity(output);
        break;}
    }

    double tpower = LF.get_target_velocity(); //Speed sent to motors
    double rpower = LF.get_actual_velocity(); //Actual speed of the motors

    if(IMUHeading > 355){
      IMUHeading = 0;
    } //Zeroes the average so it has a zero position
    if(IMUHeading >= theta-tol && IMUHeading <= theta+tol){ //If it gets really close to the wanted angle it breaks the loop
      isSettled = true;
      justPID = false;
      leftFrontMotor.move(0);
      leftBackMotor.move(0);
      rightFrontMotor.move(0);
      rightBackMotor.move(0);
      break;
    }
  pros::delay(15);
  }
return *this;
}

driveBase& driveBase::drive(double target) {
    odomReset();

    double leftvalue = LOdometer.get_position(); //LEncoder.get_value();
    double rightvalue = ROdometer.get_position();  //REncoder.get_value();
    printf("Left, Right %f %f  \n", leftvalue, rightvalue);

    isSettled = false;

    while(true) {

        leftvalue = (LOdometer.get_position())/100; //LEncoder.get_value();
        rightvalue =(ROdometer.get_position())/100;  //REncoder.get_value();

        double averagePos = (leftvalue+rightvalue)/2;//(REncoder.get_value() + LEncoder.get_value())/2;
        double error = target - averagePos;
        double derivative = error - prevError;
        prevError = error;
        double power = error*kP_drive + derivative*kD_drive;

        if(output < power && !justPID) {
            if(target > 0) {
                output = fabs(((-2*slew_a)/pow(M_E, (2.2*slew_x)/slew_a)+1)+slew_a);
            } else {
                output = ((-2*slew_a)/pow(M_E, (2.2*slew_x)/slew_a)+1)+slew_a;
            }
        } else {
            output = power;
        } if(justPID) {
            output = power;
        }

        slew_x += 0.001;

        double LOutput = output;
        double ROutput = output;

        IMUHeading = inertial.get_heading();

        if(IMUHeading > 355) {
            IMUHeading = 0;
        }
        if(IMUHeading != drive_theta){ //Corrects the robot if it is straying from the wanted angle.
            int headDifference = drive_theta - IMUHeading;
            switch (drive_theta){
                case 0: {
                    if(abs(headDifference) < 180){
                    LOutput -= LOutput * correction_rate;
                    ROutput += ROutput * correction_rate;
                    }else{
                    LOutput += LOutput * correction_rate;
                    ROutput -= ROutput * correction_rate;
                    }
                    break;
                }
                default:{
                    // if(headDifference < drive_theta){
                    if(IMUHeading < drive_theta && IMUHeading!=0){
                    LOutput += LOutput * correction_rate;
                    ROutput -= ROutput * correction_rate;
                    }
                    // else if(headDifference > drive_theta){
                    else if(IMUHeading > drive_theta && IMUHeading!=0){
                    LOutput -= LOutput * correction_rate;
                    ROutput += ROutput * correction_rate;
                    }
                    if(headDifference >180 && IMUHeading ==0){
                    LOutput -= LOutput * correction_rate;
                    ROutput += ROutput * correction_rate;
                    }else if(headDifference <180 && IMUHeading ==0){
                    LOutput += LOutput * correction_rate;
                    ROutput -= ROutput * correction_rate;
                    }
                    break;
                }
            }
        }

        rightFrontMotor.move_velocity(ROutput);
        rightBackMotor.move_velocity(ROutput);
        leftFrontMotor.move_velocity(LOutput);
        leftBackMotor.move_velocity(LOutput);

        double leftvalue = LOdometer.get_position(); //LEncoder.get_value();
        double rightvalue = ROdometer.get_position(); //REncoder.get_value();

        pros::delay(10);

        if(averagePos < target+tol && averagePos > target-tol) {
            stop();
            break;
        }
    }
    return *this;
}

void driveBase::waitUntilSettled(bool halt_ = 1){
    halt = halt_;
}

driveBase& driveBase::withSlop(double drive_tol_ = 10, double turn_tol_ = 1){
    drive_tol = drive_tol_;
    turn_tol = turn_tol_;
    return *this;
}

driveBase& driveBase::withTurn(double theta_, double turn_kP_, double turn_kI_, double turn_kD_) {
    theta = theta_;
    turn_kP = turn_kP_;
    turn_kI = turn_kI_;
    turn_kD = turn_kD_;
    return *this;
}

driveBase& driveBase::move(double target, double drive_kP, double drive_kI, double drive_kD) {
    //Convert target from inches to encoder ticks.
    target *= CONVERSION;
    isSettled = 0;
    odomReset();
    printf("isSettled: %d \n", isSettled);

    while(!isSettled){
        // Lateral mvmt PID calc.
        double current_Left = LOdometer.get_position();//( LOdometer.get_position() )/36000; //Convert from centidegress to degrees.
        double current_Right = ROdometer.get_position();//( ROdometer.get_position )/36000;
        // current_Left /= CONVERSION;
        // current_Right /= CONVERSION;
        double averagePos = ( current_Left + current_Right ) /2;

        m_error = target - averagePos;
        m_integral += m_error;

        if(m_error == 0) {
            m_integral = 0;
        }
        if(m_integral > 12000) {
            m_integral = 120000;
        }

        m_derivative = m_error - m_prevError;
        m_prevError = m_error;
        m_power = ( (m_error * drive_kP) + (m_integral * drive_kI) + (m_derivative * drive_kD) );
        
        if(m_power > 12000) {
            m_power = 12000;
        }

        LOutput = m_power;
        ROutput = m_power;

        // Turn mvmt PID calc.
        IMUHeading = inertial.get_heading();

        if(IMUHeading > 358){
            if(direction_turn ==LEFT){
                IMUHeading=360;
            } else {
                IMUHeading = 0;
            }
        }

        t_error = IMUHeading - theta;
        t_integral += t_error;

        if(t_error == 0){
            t_integral = 0;
        }
        if(t_integral > 12000){
            t_integral = 120000;
        }

        t_derivative = t_error - t_prevError;
        t_prevError = t_error;
        turn_output = (t_error * turn_kP) + (t_integral * turn_kI) + (t_derivative * turn_kD);
        
        calcDir(IMUHeading, theta);

        switch(direction_turn){
            case LEFT:{
                if(m_error < 0){
                    LOutput += fabs(turn_output);
                    ROutput -= fabs(turn_output);
                } else{
                    LOutput -= fabs(turn_output);
                    ROutput += fabs(turn_output);
                }
                break;
            }
            case RIGHT: {
                if(m_error < 0){
                    LOutput += fabs(turn_output);
                    ROutput -= fabs(turn_output);
                } else {
                    LOutput += fabs(turn_output);
                    ROutput -= fabs(turn_output);
                }
                break;
            }
        }

        leftFrontMotor.move_voltage(LOutput);
        leftBackMotor.move_voltage(LOutput);
        rightFrontMotor.move_voltage(ROutput);
        rightBackMotor.move_voltage(ROutput);

        if(fabs(m_error) < drive_tol && fabs(t_error) < turn_tol){
            stop();
            break;
        }
        pros::delay(2);
    }
    return *this;
}
