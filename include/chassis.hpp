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
    
        void move(float distance, float maxSpeed=127);
        void turn(float heading, float maxSpeed=127);
        void swing(float heading, bool isLeft, float maxSpeed=127);
        void turnToPoint(float x1, float y1, int timeout, float maxSpeed=127);
        void move_without_settle(float distance, float timeout);
        void swing_without_settle(float heading, bool isLeft, float timeout);
        void moveToPoint(float x1, float y1, int timeout, float maxSpeed=127);
        void moveToPointTurn(float x1, float y1, int timeout, float maxSpeed=127);
        void arc(float heading, double leftMult, double rightMult);
        void setPose(float x1, float y1, float theta1);
        void activeMoveToPoint(float x1, float y1, int timeout, float maxSpeed=127);
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
        float angleError(float angle1, float angle2, bool radians);
        const float wheelDiameter;
        const float gearRatio;
        
};