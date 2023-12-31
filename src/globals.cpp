#include "globals.hpp"

pros::IMU inertial(12);

pros::MotorGroup leftMotors(std::initializer_list<pros::Motor> {
    pros::Motor(-15, pros::E_MOTOR_GEAR_BLUE), // left front motor. port 8, reversed
    pros::Motor(-11, pros::E_MOTOR_GEAR_BLUE), // left middle motor. port 20, reversed
    pros::Motor(-14, pros::E_MOTOR_GEAR_BLUE),
});

pros::MotorGroup rightMotors(std::initializer_list<pros::Motor> {
    pros::Motor(16, pros::E_MOTOR_GEAR_BLUE),
    pros::Motor(17, pros::E_MOTOR_GEAR_BLUE),
    pros::Motor(18, pros::E_MOTOR_GEAR_BLUE),
});


// left motors, right motors, inertial, wheelDiameter, gear Ratio, lateralPID, turnPID, swingPID
Chassis chassis(
    &leftMotors, &rightMotors, &inertial, 
    2.75, 1, 
    //lateral
    PID(10, 0, 45, 0, 0.75, 250, 2, 750, 7000),
    PID(10, 0, 45, 0, 0.75, 250, 1, 750, 7000),
    //angular 
    PID(3.5, 0, 15, 0, 1, 100, 3, 250, 1000),

    PID(3.5, 0, 15, 0, 1, 100, 3, 250, 1000),
    PID(6, 0, 50, 0, 1, 100, 3, 250, 4000),
    PID(5, 0, 15, 0, 1, 100, 3, 250, 1000)
);



pros::Controller master(pros::E_CONTROLLER_MASTER);