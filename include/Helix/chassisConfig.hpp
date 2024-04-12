#pragma once

#include <functional>
#include <cmath>
#include "api.h"

namespace Helix {
    ////////////////////////////////////////////////////////////////////////
    // Contains all the important structures for drive and other settings //
    ////////////////////////////////////////////////////////////////////////

struct Drivetrain {
    /**
     * @param leftMotors pointer to the left motors
     * @param rightMotors pointer to the right motors
     * @param trackWidth the track width of the robot
     * @param wheelDiameter the diameter of the wheel used on the drivetrain
     * @param rpm the rpm of the wheels
     * @param chasePower higher values make the robot move faster but causes more overshoot on turns
     */
     Drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, float trackWidth, float wheelDiameter,
     float rpm, float chasePower);
     pros::Motor_Group* leftMotors;
     pros::Motor_Group* rightMotors;
     float trackWidth;
     float wheelDiameter;
     float rpm;
     float chasePower;
};

struct Sensors {
    /**
     * Allows easier access to imputing the IMU sensor
     * @param imu pointer to the IMU
     */
    Sensors(pros::Imu* imu);
    pros::Imu* imu;
};

class Chassis {
    public:
        /**
         * @brief Construct a new chassis
         *
         * @param drivetrain Drivetrain to be used for the chassis
         * @param sensors Sensors to be used for the PID
         */
        Chassis(Drivetrain drivetrain, Sensors sensors);

}; 

}