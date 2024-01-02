#include "globals.hpp"


//inertial sensor
pros::IMU inertial(12);

//edit
const float wheelDiam = 2.75;
const float trackWidth = 13;
const float externalGearRatio = 1;

//left motors
pros::MotorGroup leftMotors(std::initializer_list<pros::Motor> {
    pros::Motor(-15, pros::E_MOTOR_GEAR_BLUE), // left front motor. port 8, reversed
    pros::Motor(-11, pros::E_MOTOR_GEAR_BLUE), // left middle motor. port 20, reversed
    pros::Motor(-14, pros::E_MOTOR_GEAR_BLUE),
});

//right motors
pros::MotorGroup rightMotors(std::initializer_list<pros::Motor> {
    pros::Motor(16, pros::E_MOTOR_GEAR_BLUE),
    pros::Motor(17, pros::E_MOTOR_GEAR_BLUE),
    pros::Motor(18, pros::E_MOTOR_GEAR_BLUE),
});



Chassis chassis(
    //left motors, right motors, inertial sensor
    &leftMotors, &rightMotors, &inertial, 
    //wheel diameter, track width, gear ratio
    wheelDiam, trackWidth, externalGearRatio, 
    //forward PID
    PID(10, 0, 45, 0, 0.75, 250, 2, 750, 7000),
    //backward PID
    PID(10, 0, 45, 0, 0.75, 250, 1, 750, 7000),
    //turnPID
    PID(3.5, 0, 15, 0, 1, 100, 3, 250, 1000),
    //swing PID
    PID(3.5, 0, 15, 0, 1, 100, 3, 250, 1000),
    //arc PID
    PID(6, 0, 50, 0, 1, 100, 3, 250, 4000),
    //heading PID
    PID(5, 0, 15, 0, 1, 100, 3, 250, 1000)
);
//that's all it takes for integrated encoder odom!


pros::Controller master(pros::E_CONTROLLER_MASTER);