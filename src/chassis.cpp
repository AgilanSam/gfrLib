#include "chassis.hpp"

#include "pros/motors.hpp"
#include "pros/llemu.hpp"
#include "api.h"
#include <cmath>
#include <iostream>


Chassis::Chassis(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, pros::IMU* inertial, const float wheelDiameter,
                 const float gearRatio, PID drivePID, PID backwardPID, PID turnPID, PID swingPID, PID arcPID, PID headingPID)
    : leftMotors(leftMotors), rightMotors(rightMotors), imu(inertial), wheelDiameter(wheelDiameter), gearRatio(gearRatio),
      drivePID(drivePID), backwardPID(backwardPID), turnPID(turnPID), swingPID(swingPID), arcPID(arcPID),headingPID(headingPID){
    this->leftMotors->set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
    this->rightMotors->set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
}

void Chassis::calibrate() {
    pros::Task calibratetask([=]{
        imu->reset();
        double prev_forward_travel = 0.0;
        double previous_heading = 0.0;
        double xval;
        double yval;
        double deltaX;
        double deltaY;
    while (true) {
        //get current vals
        double forward_travel = ((leftMotors->get_positions()[0]+rightMotors->get_positions()[0])/2)* (wheelDiameter*M_PI) * gearRatio;
        double heading_radians = imu->get_heading() * (M_PI / 180.0);
        double average_heading = (previous_heading + heading_radians) / 2;
        double delta_forward_travel = forward_travel - prev_forward_travel;
        x = forward_travel * std::sin(average_heading);
        y = forward_travel * std::cos(average_heading);
        heading = imu->get_heading();
        prev_forward_travel = forward_travel;
        previous_heading = heading_radians;
        

        pros::delay(10);

    }
    pros::delay(10);
    });
    
    
}

void Chassis::setHeading(float heading) {
    imu->set_heading(heading);
}

void Chassis::tank(float left, float right) {
    leftMotors->move(left);
    rightMotors->move(right);
}

void Chassis::arcade(float lateral, float angular) {
    leftMotors->move(lateral + angular);
    rightMotors->move(lateral - angular);
}


static float rollAngle180(float angle) {
    while (angle < -180) {
        angle += 360;
    }

    while (angle >= 180) {
        angle -= 360;
    }

    return angle;
}

void Chassis::move(float distance) {
    drivePID.reset();
    backwardPID.reset();
    headingPID.reset();
    float beginningLeft = leftMotors->get_positions()[0];
    float beginningRight = rightMotors->get_positions()[0];
    float angle = imu->get_heading();
    if(distance < 0){
        do {
        float deltaLeft = leftMotors->get_positions()[0] - beginningLeft;
        float deltaRight = rightMotors->get_positions()[0] - beginningRight;
        float distanceTravelled = (deltaLeft + deltaRight) / 2 * wheelDiameter * M_PI * gearRatio;
        float error = rollAngle180(angle - imu->get_heading());
        float pidAngOutput = headingPID.update(0, -error);
        float pidOutputLateral = backwardPID.update(distance, distanceTravelled);
        arcade(pidOutputLateral, pidAngOutput);

        pros::delay(20);
    } while (!backwardPID.isSettled());
    } else{
        do {
        float deltaLeft = leftMotors->get_positions()[0] - beginningLeft;
        float deltaRight = rightMotors->get_positions()[0] - beginningRight;
        float distanceTravelled = (deltaLeft + deltaRight) / 2 * wheelDiameter * M_PI * gearRatio;
        float error = rollAngle180(angle - imu->get_heading());
        float pidAngOutput = headingPID.update(0, -error);
        float pidOutputLateral = drivePID.update(distance, distanceTravelled);
        arcade(pidOutputLateral, pidAngOutput);

        pros::delay(20);
    } while (!drivePID.isSettled());
    }

    arcade(0, 0);
}
void Chassis::turn(float heading) {
    turnPID.reset();

    do {
        float error = rollAngle180(heading - imu->get_heading());
        float pidOutput = turnPID.update(0, -error);

        arcade(0, pidOutput);

        pros::lcd::print(1, "heading: %f", imu->get_heading());

        pros::delay(20);
    } while (!turnPID.isSettled());

    arcade(0, 0);
}



void Chassis::swing(float heading, bool isLeft){
    swingPID.reset();
    if(isLeft){
        do {
        float error = rollAngle180(heading - imu->get_heading());
        float pidOutput = swingPID.update(0, -error);
        Chassis::rightMotors->set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
        tank(pidOutput, 0);

        pros::lcd::print(1, "heading: %f", imu->get_heading());

        pros::delay(20);
    } while (!swingPID.isSettled());
    arcade(0, 0);
    } else{
        do {
        float error = rollAngle180(heading - imu->get_heading());
        float pidOutput = swingPID.update(0, -error);
        Chassis::leftMotors->set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
        tank(0, pidOutput);

        pros::lcd::print(1, "heading: %f", imu->get_heading());

        pros::delay(20);
    } while (!swingPID.isSettled());
    arcade(0, 0);
    }
}


void Chassis::move_without_settle(float distance, float timeout){
    drivePID.reset();

    float beginningLeft = leftMotors->get_positions()[0];
    float beginningRight = rightMotors->get_positions()[0];

    auto start = pros::millis();
    do {
        float deltaLeft = leftMotors->get_positions()[0] - beginningLeft;
        float deltaRight = rightMotors->get_positions()[0] - beginningRight;
        float distanceTravelled = (deltaLeft + deltaRight) / 2 * wheelDiameter * M_PI * gearRatio;

        float pidOutput = drivePID.update(distance, distanceTravelled);
        arcade(pidOutput, 0);

        pros::delay(20);
    } while ((pros::millis() - start) != timeout);
    arcade(0,0);
    leftMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    rightMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
}
//try

void Chassis::arc(float heading, double leftMult, double rightMult){
    arcPID.reset();
        do {
        float error = rollAngle180(heading - imu->get_heading());
        float pidOutput = arcPID.update(0, -error);
        if(fabs(pidOutput) > 127) pidOutput = 127;
        if(error < 1) break;
        tank(pidOutput * leftMult, pidOutput * rightMult);
        
        pros::lcd::print(1, "heading: %f", imu->get_heading());

        pros::delay(20);
    } while (!arcPID.isSettled());
    arcade(0, 0);
    leftMotors->set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
    rightMotors->set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
    }


void Chassis::swing_without_settle(float heading, bool isLeft, float timeout){
    swingPID.reset();
    if(isLeft){
        auto start = pros::millis();
        do {
        float error = rollAngle180(heading - imu->get_heading());
        float pidOutput = swingPID.update(0, -error);

        tank(pidOutput, 0);

        pros::lcd::print(1, "heading: %f", imu->get_heading());

        pros::delay(10);
    } while ((pros::millis() - start) != timeout);
    arcade(0,0);
    leftMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    rightMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    } else{
        auto start = pros::millis();
        do {
        float error = rollAngle180(heading - imu->get_heading());
        float pidOutput = swingPID.update(0, -error);
        
        tank(0, pidOutput);

        pros::lcd::print(1, "heading: %f", imu->get_heading());

        pros::delay(10);
    } while ((pros::millis() - start) != timeout);
    arcade(0,0);
    leftMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    rightMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    }
}
float distance(int x1, int y1, int x2, int y2) 
{ 
    // Calculating distance 
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) * 1.0); 
} 

//odom movements
float radToDeg(float rad) { return rad * 180 / M_PI; }
float degToRad(float deg) { return deg * M_PI / 180; }

void Chassis::moveToPoint(float x1, float y1, int timeout, float maxSpeed){
        //turn part
        float angleError = atan2(y1 - y, x1 - x);
        turn(rollAngle180(-radToDeg(angleError)));
        float lateralError = distance(x,y, x1,y1);
        move(lateralError);
    
}
    
void Chassis::turnToPoint(float x1, float y1, int timeout, float maxSpeed){
        //turn part
        float angleError = atan2(y1 - y, x1 - x);
        turn(rollAngle180(-radToDeg(angleError)));
    
}