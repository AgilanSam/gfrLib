#include "chassis.hpp"

#include "pros/motors.hpp"
#include "pros/llemu.hpp"
#include "api.h"
#include <cmath>
#include <iostream>

#define M_PI		3.14159265358979323846
#define M_PI_2		1.57079632679489661923
#define M_PI_4		0.78539816339744830962
#define M_1_PI		0.31830988618379067154
#define M_2_PI		0.63661977236758134308

Chassis::Chassis(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, pros::IMU* inertial, const float wheelDiameter,
                 const float gearRatio, PID drivePID, PID backwardPID, PID turnPID, PID swingPID, PID arcPID, PID headingPID)
    : leftMotors(leftMotors), rightMotors(rightMotors), imu(inertial), wheelDiameter(wheelDiameter), gearRatio(gearRatio),
      drivePID(drivePID), backwardPID(backwardPID), turnPID(turnPID), swingPID(swingPID), arcPID(arcPID),headingPID(headingPID){
    this->leftMotors->set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
    this->rightMotors->set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
}
void Chassis::waitUntilDist(float dist) {
    // do while to give the thread time to start
    do pros::delay(10);
    while (distTravelled <= dist && distTravelled != -1);
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
    double leftmotorsmove = lateral + angular;
    double rightmotorsmove = lateral - angular;
    leftMotors->move(leftmotorsmove);
    rightMotors->move(rightmotorsmove);
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

void Chassis::move(float distance, float maxSpeed) {
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
        if (pidOutputLateral<-maxSpeed) {
            pidOutputLateral=-maxSpeed; 
        }
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
        if (pidOutputLateral>maxSpeed) {
            pidOutputLateral=maxSpeed; 
        }
        arcade(pidOutputLateral, pidAngOutput);

        pros::delay(20);
    } while (!drivePID.isSettled());
    }

    arcade(0, 0);
}
void Chassis::turn(float heading,float maxSpeed) {
    turnPID.reset();

    do {
        float error = rollAngle180(heading - imu->get_heading());
        float pidOutput = turnPID.update(0, -error);
        if (pidOutput>maxSpeed) {
            pidOutput=maxSpeed; 
        } else if (pidOutput<-maxSpeed) {
            pidOutput=-maxSpeed; 
        } else {
            pidOutput = pidOutput;
        }
        arcade(0, pidOutput);

        pros::lcd::print(1, "heading: %f", imu->get_heading());

        pros::delay(20);
    } while (!turnPID.isSettled());

    arcade(0, 0);
}



void Chassis::swing(float heading, bool isLeft, float maxSpeed){
    swingPID.reset();
    if(isLeft){
        do {
        float error = rollAngle180(heading - imu->get_heading());
        float pidOutput = swingPID.update(0, -error);
        Chassis::rightMotors->set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
        if (pidOutput>maxSpeed) {
            pidOutput=maxSpeed; 
        } else if (pidOutput<-maxSpeed) {
            pidOutput=-maxSpeed; 
        } else {
            pidOutput = pidOutput;
        }
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
        if (pidOutput>maxSpeed) {
            pidOutput=maxSpeed; 
        } else if (pidOutput<-maxSpeed) {
            pidOutput=-maxSpeed; 
        } else {
            pidOutput = pidOutput;
        }
        tank(0, pidOutput);

        pros::lcd::print(1, "heading: %f", imu->get_heading());

        pros::delay(20);
    } while (!swingPID.isSettled());
    arcade(0, 0);
    }
}



//try
void Chassis::move_without_settle(float distance, float exitrange){
    drivePID.reset();
    float beginningLeft = leftMotors->get_positions()[0];
    float beginningRight = rightMotors->get_positions()[0];
    float error;
    auto start = pros::millis();
    do {
        float deltaLeft = leftMotors->get_positions()[0] - beginningLeft;
        float deltaRight = rightMotors->get_positions()[0] - beginningRight;
        float distanceTravelled = (deltaLeft + deltaRight) / 2 * wheelDiameter * M_PI * gearRatio;
        error = distance-distanceTravelled;
        float pidOutput = drivePID.update(distance, distanceTravelled);
        arcade(pidOutput, 0);

        pros::delay(20);
    } while (!drivePID.isSettled() || fabs(error) > exitrange); 
    arcade(0,0);
    leftMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    rightMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
}

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
float distance(double x1, double y1, double x2, double y2) 
{ 
    // Calculating distance 
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) * 1.0); 
} 

float pointAngleDifference(double x1, double y1, double x2, double y2){
    return std::atan2(y2- y1, x2 - x1); 
}
//odom movements
float radToDeg(float rad) { return rad * 180 / M_PI; }
float degToRad(float deg) { return deg * M_PI / 180; }
void Chassis::setPose(float x1, float y1, float theta1) {
    x = x1;
    y = y1;
    imu->set_heading(theta1);
}
std::pair<double, double> Chassis::getPose(){
    return std::make_pair(x, y);

} 
void Chassis::moveToPoint(float x1, float y1, int timeout, float maxSpeed){
        //turn part
        float angleError = atan2(y - y1, x - x1); //can flip this
        turn(rollAngle180(radToDeg(angleError))); //was negative
        float lateralError = distance(x,y, x1,y1);
        move(lateralError, maxSpeed);
    
}
    
void Chassis::turnToPoint(float x1, float y1, int timeout, float maxSpeed){
        //turn part
        float angleError = atan2(y - y1, x - x1);
        turn(rollAngle180(radToDeg(angleError)), maxSpeed);
    
}
// void Chassis::moveToPointconstant(float x1, float y1, int timeout, float maxSpeed){
//         //turn part
//         maxSpeedglobal = maxSpeed;
//         float angleError = atan2(y - y1, x - x1); //can flip this
//         float lateralError = distance(x,y, x1,y1);
//         turn(rollAngle180(radToDeg(angleError))); //was negative
//         float pidOutputLateral = backwardPID.update(distance, distanceTravelled);
//         arcade(pidOutputLateral, pidAngOutput);
//         move(lateralError);
    
// }
float Chassis::angleError(float angle1, float angle2, bool radians) {
    float max = radians ? 2 * M_PI : 360;
    float half = radians ? M_PI : 180;
    angle1 = fmod(angle1, max);
    angle2 = fmod(angle2, max);
    float error = angle1 - angle2;
    if (error > half) error -= max;
    else if (error < -half) error += max;
    return error;
}
int sgn(float x) {
    if (x < 0) return -1;
    else return 1;
}
float getCurvature(double posex, double posey, double posetheta, double otherx, double othery, double othertheta) {
    // calculate whether the pose is on the left or right side of the circle
    float side = sgn(std::sin(posetheta) * (otherx - othery) - std::cos(posetheta) * (othery - posey));
    // calculate center point and radius
    float a = -std::tan(posetheta);
    float c = std::tan(posetheta) * posex - posey;
    float x = std::fabs(a * otherx + othery + c) / std::sqrt((a * a) + 1);
    float d = std::hypot(otherx - posex, othery - posey);

    // return curvature
    return side * ((2 * x) / (d * d));
}
void Chassis::activeMoveToPoint(float x1, float y1, int timeout, float maxSpeed){
    turnPID.reset();
    drivePID.reset();
    
    float prevLateralPower = 0;
    float prevAngularPower = 0;
    bool close = false;
    uint32_t start = pros::millis();
    
    while(((start < timeout) || (!drivePID.isSettled() && !turnPID.isSettled()))){
        heading = std::fmod(heading, 360);

        //update error
        float deltaX = x1 - x;
        float deltaY = y1 - y;
        float targetTheta = fmod(radToDeg(M_PI_2 - atan2(deltaY, deltaX)), 360);
        float hypot = std::hypot(deltaX, deltaY);
        float diffTheta1 = angleError(heading, targetTheta, false);
        float diffTheta2 = angleError(heading, targetTheta + 180, false);
        float angularError = (std::fabs(diffTheta1) < std::fabs(diffTheta2)) ? diffTheta1 : diffTheta2;
        float lateralError = hypot * cos(degToRad(std::fabs(diffTheta1)));
        float lateralPower = drivePID.update(lateralError, 0);
        float angularPower = -turnPID.update(angularError, 0);

        if (distance(x1, y1, x, y) < 7.5) {
             close = true;
             maxSpeed = (std::fabs(prevLateralPower) < 30) ? 30 : std::fabs(prevLateralPower);
         }
        if (lateralPower > maxSpeed) lateralPower = maxSpeed;
        else if (lateralPower < -maxSpeed) lateralPower = -maxSpeed;
        if (close) angularPower = 0;

        prevLateralPower = lateralPower;
        prevAngularPower = angularPower;

        float leftPower = lateralPower + angularPower;
        float rightPower = lateralPower - angularPower;

        float ratio = std::max(std::fabs(leftPower), std::fabs(rightPower)) / maxSpeed;
        if (ratio > 1) {
            leftPower /= ratio;
            rightPower /= ratio;
        }
        tank(leftPower, rightPower);
        pros::delay(10);
    }
    tank(0,0);

}
void Chassis::moveToPose(float x1, float y1, float theta1, int timeout, bool forwards, float maxSpeed, bool async,float chasePower,
                          float lead, float smoothness, bool linearexit, float linearexitrange) {
    if (!mutex.take(10)) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { moveToPose(x1, y1, theta1, timeout, forwards, maxSpeed, async, chasePower,
                           lead,  smoothness,  linearexit,  linearexitrange); });
        mutex.give();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    turnPID.reset();
    drivePID.reset();
    
    double targetTheta = M_PI_2 - degToRad(theta1);
    float prevLateralPower = 0;
    float prevAngularPower = 0;

    //last pose
    float lastposex = x;
    float lastposey = y;
    float lastposetheta = heading;

    auto start = pros::millis();
    if (!forwards) targetTheta = fmod(targetTheta + M_PI, 2 * M_PI); // backwards movement

    bool close = false;
    if(chasePower ==  0)chasePower = 40; // make chasePower globalized in chassis setup
    while ((!drivePID.isSettled() || pros::millis() - start < timeout)) {
        double currX = x;
        double currY = y;
        double currHeading = heading; //for reference
        double currTheta = degToRad(heading);

        if (!forwards) currTheta += M_PI;
        distTravelled += distance(lastposex, lastposey, currX, currY); 

        //update prev
        lastposex = currX;
        lastposey = currY;
        lastposetheta = currHeading;

        if (distance(x1, y1, x, y) < 7.5) {
            close = true;
        }

        //carrot - 2 times for each part
        double carrotX = x1 - (cos(targetTheta) * lead * distance( x1, y1,currX, currY));
        double carrotY = y1 - (sin(targetTheta) * lead * distance( x1, y1,currX, currY));   
        if(close){ //settle behavior
            x1 = carrotX;
            y1 = carrotY;
        }

        // calculate error
        float angularError = angleError(pointAngleDifference(carrotX, carrotY, currX, currY), currTheta, true); // angular error
        float linearError = distance(carrotX, carrotY,currX, currY) * cos(angularError); // linear error
        if (linearexit=true && fabs(linearError)>linearexitrange) {
            Chassis::leftMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
            Chassis::rightMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
            break;
        } 
        if (close) angularError = angleError(targetTheta, currTheta, true); // settling behavior
        if (!forwards) linearError = -linearError;
        
        // get PID outputs
        float angularPower = -turnPID.update(radToDeg(angularError), 0);
        float linearPower = drivePID.update(linearError, 0);
        float rescale = linearPower + fabs(angularPower) - 127.0 - 127.0 * smoothness; 
        if (rescale>0) linearPower -=rescale; 
        
        float curvature = fabs(getCurvature(currX,currY, currTheta, carrotX, carrotY, 0));
        if (curvature == 0) curvature = -1;
        float radius = 1 / curvature;

        // calculate the maximum speed at which the robot can turn
        // using the formula v = sqrt( u * r * g )
        if (radius != -1) {
            float maxTurnSpeed = sqrt(chasePower * radius * 9.8);
            // the new linear power is the minimum of the linear power and the max turn speed
            if (linearPower > maxTurnSpeed && !close) linearPower = maxTurnSpeed;
            else if (linearPower < -maxTurnSpeed && !close) linearPower = -maxTurnSpeed;
        }
        // prioritize turning over moving
        float overturn = fabs(angularPower) + fabs(linearPower) - maxSpeed;
        if (overturn > 0) linearPower -= linearPower > 0 ? overturn : -overturn;

        // calculate motor powers
        float leftPower = linearPower + angularPower;
        float rightPower = linearPower - angularPower;

        tank(leftPower, rightPower);
        pros::delay(10);

    } 
    
    tank(0,0);
    distTravelled = -1;
    mutex.give();
}

void Chassis::move_without_settletime(float distance, float timeout){
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
