//CONTROLLER
pros::Controller master(CONTROLLER_MASTER);

//MOTORS
pros::Motor rightFrontMotor(1, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_COUNTS);
pros::Motor rightBackMotor(2, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_COUNTS);
pros::Motor leftFrontMotor(1, MOTOR_GEARSET_18, 1, MOTOR_ENCODER_COUNTS);
pros::Motor leftBackMotor(4, MOTOR_GEARSET_18, 1, MOTOR_ENCODER_COUNTS);



