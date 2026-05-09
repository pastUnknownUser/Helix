#pragma once
#include "pros/motor_group.hpp"
#include "PID.h"

namespace Helix {
    class Chassis {
        private:
            pros::MotorGroup& leftSide;
            pros::MotorGroup& rightSide;
            
            PID drivePID;

            double wheelDiameter;
            double gearRatio;

        public:
            Chassis(pros::MotorGroup& left, pros::MotorGroup& right, double wheel_diameter, double ratio);
            
            void setDrivePID(double kP, double kI, double kD, double max_i, double max_out, double tolerance);

            void move(double targetInches);

    };
}