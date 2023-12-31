#pragma once

#include "pros/motors.hpp"
#include "pros/imu.hpp"
#include "pose.hpp"
#include "pid.hpp"

class Chassis {
    public:
        
        PID drivePID;
        PID turnPID;
        PID swingPID;
        PID headingPID;
        PID backwardPID;
        PID arcPID;


        Chassis(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, pros::IMU* inertial,
                const float wheelDiameter, const float gearRatio, PID drivePID, PID backwardPID, PID turnPID, PID swingPID, PID arcPID, PID headingPID);

        void calibrate();

        void setHeading(float heading);

        void tank(float left, float right);
        void arcade(float lateral, float angular);
    
        void move(float distance);
        void turn(float heading);
        void swing(float heading, bool isLeft);
        void turnToPoint(float x1, float y1, int timeout, float maxSpeed);
        void move_without_settle(float distance, float timeout);
        void swing_without_settle(float heading, bool isLeft, float timeout);
        void moveToPoint(float x1, float y1, int timeout, float maxSpeed);
        void moveToPointTurn(float x1, float y1, int timeout, float maxSpeed);
        void arc(float heading, double leftMult, double rightMult);
        void setPose(float x1, float y1, float theta1);
        std::pair<double, double> getPose();
        float get_absolute_heading();
        float reduce_0_to_360(float angle);
        double x;
        double y;
        double heading = 0;
    private:
        pros::MotorGroup* leftMotors;
        pros::MotorGroup* rightMotors;
        pros::IMU* imu;

        const float wheelDiameter;
        const float gearRatio;
        
};