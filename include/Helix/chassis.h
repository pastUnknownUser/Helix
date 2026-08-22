#pragma once

#include "PID.h"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"

namespace Helix {
    class Chassis {
    private:
        pros::MotorGroup& leftSide;
        pros::MotorGroup& rightSide;
        pros::Imu& imu;

        PID drivePID;
        PID turnPID;

        double wheelDiameter;
        double gearRatio;

    public:
        Chassis(pros::MotorGroup& left, pros::MotorGroup& right, pros::Imu& imu,
                double wheel_diameter, double external_gear_ratio);

        void setDrivePID(double kP, double kI, double kD, double max_i,
                         double max_out, double tolerance);
        void setTurnPID(double kP, double kI, double kD, double max_i,
                        double max_out, double tolerance);

        void move(double targetInches);
        void turn(double targetDegrees);
    };
}
