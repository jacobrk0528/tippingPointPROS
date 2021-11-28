#include "drivebase.hpp"

bool DriveBase::isSettled = true;
bool DriveBase::justPID = false;
bool DriveBase::oneSide;
bool DriveBase::halt;

double DriveBase::IMUHeading = inertial.get_heading();
double DriveBase::power;
double DriveBase::kP_drive, DriveBase::kD_drive, DriveBase::kP_turn, DriveBase::kD_turn;
double DriveBase::rate_drive, DriveBase::rate_turn, DriveBase::correction_rate;
double DriveBase::output = 1;
double DriveBase::prevError = 0;
double DriveBase::turnPrevError = 0;
double DriveBase::m_error, DriveBase::m_integral, DriveBase::m_derivative, DriveBase::m_prevError, DriveBase::m_power, DriveBase::LOutput, DriveBase::ROutput, DriveBase::drive_tol = 10, DriveBase::turn_tol = 1, DriveBase::t_error, DriveBase::t_integral, DriveBase::t_derivative, DriveBase::t_prevError, DriveBase::theta, DriveBase::turn_kP, DriveBase::turn_kI, DriveBase::turn_kD, DriveBase::turn_output;

int DriveBase::drive_theta;
int DriveBase::direction_turn;
int DriveBase::slew_a = 600, DriveBase::slew_x = 1;
int DriveBase::tol, DriveBase::heading_diff;



void DriveBase::reset() {
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

void DriveBase::odomReset(){
    LOdometer.reset_position();
    ROdometer.reset_position();
}

void DriveBase::setBreak(int breakState){
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

void DriveBase::stop() {
    rightFrontMotor.move(0);
    rightBackMotor.move(0);
    leftFrontMotor.move(0);
    leftBackMotor.move(0);

    odomReset();

    justPID = false;
}

DriveBase& DriveBase::withTurnSlew(int turnSlewRate) {
    rate_turn = turnSlewRate;
    return *this;
}

DriveBase& DriveBase::withTurnPD(double kP_, double kD_) {
    kP_turn = kP_;
    kD_turn = kD_;
    return *this;
}

DriveBase& DriveBase::withTurnDirection(int turnDirection) {
     direction_turn = turnDirection;
     return *this;
}

DriveBase& DriveBase::withSlew(int slewRate) {
    slew_a = slewRate;
    return *this;
}

DriveBase& DriveBase::withPD(double kP_, double kD_) {
    kP_drive = kP_;
    kD_drive = kD_;
    return *this;
}

DriveBase& DriveBase::withHeading(double driveAngle, double correctionRate) {
    drive_theta = driveAngle;
    correction_rate = correctionRate;
    return *this;
}

DriveBase& DriveBase::justPD(bool justPD_) {
    justPID = justPD_;
    return *this;
}

void DriveBase::calcTurnDirection(int currentPos, int targetPos) {
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

DriveBase& DriveBase::turn(double desiredTurnAngle) {
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

    calcTurnDirection(IMUHeading, desiredTurnAngle);

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

    double tpower = leftFrontMotor.get_target_velocity(); //Speed sent to motors
    double rpower = leftBackMotor.get_actual_velocity(); //Actual speed of the motors

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

DriveBase& DriveBase::drive(double target) {
    odomReset();

    double leftvalue = LOdometer.get_position(); //LEncoder.get_value();
    double rightvalue = ROdometer.get_position();  //REncoder.get_value();
    printf("Left, Right %f %f  \n", leftvalue, rightvalue);

    isSettled = false;

    while(true) {

        leftvalue = (LOdometer.get_position())/100.0; //LEncoder.get_value();
        rightvalue =(ROdometer.get_position())/100.0;  //REncoder.get_value();

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

void DriveBase::waitUntilSettled(bool halt_){
    halt = halt_;
}

DriveBase& DriveBase::withSlop(double drive_tol_, double turn_tol_){
    drive_tol = drive_tol_;
    turn_tol = turn_tol_;
    return *this;
}

DriveBase& DriveBase::withTurn(double theta_, double turn_kP_, double turn_kI_, double turn_kD_) {
    theta = theta_;
    turn_kP = turn_kP_;
    turn_kI = turn_kI_;
    turn_kD = turn_kD_;
    return *this;
}

DriveBase& DriveBase::move(double target, double drive_kP, double drive_kI, double drive_kD) {
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
        
        calcTurnDirection(IMUHeading, theta);

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
