#pragma once

#include <functional>
#include <cmath>
#include "api.h"
#include "pros/rtos.hpp"
#include "pros/motors.hpp"
#include "pros/imu.hpp"
#include "pros/apix.h"

    ////////////////////////////////////////////////////////////////////////
    // Contains all the important structures for drive and other settings //
    ////////////////////////////////////////////////////////////////////////
    
namespace Helix {

class Drivetrain {
    public:
        /**
         * @brief The drivetrain constructor
         * 
         * @param LeftSide The left part of the drivetrain
         * @param RightSide The left part of the drivetrain
         * @param Rpm The rpm of the drive
         * @param WheelDiameter
        */ 
       Drivetrain(pros::MotorGroup* LeftSide, pros::MotorGroup* RightSide, float Rpm, float WheelDiameter);

       pros::MotorGroup* Leftside;
       pros::MotorGroup* RightSide;
       float Rpm;
       float WheelDiameter;
};

class PID {
    /**
     * @brief Constructs the Lateral PID
     * 
     * @param kP Porportional
     * @param kI Integeral
     * @param kD Derivative
     * @param integralTerm
     * @param previousError Last error reported
    */

    public:
        PID(float kP, float kI, float kD, int integralTerm, int previousError);

        float kP;
        float kI;
        float kD;
        int integralTerm;
        int previousError;
};

class Chassis {
    public:
       /**
       * @brief The config of the chassis
       * 
       * @param drivetrain Both left and right side combined
       * @param LateralSettings the PID for fwd and rev
       * 
      */

     Chassis(Drivetrain drivetrain, PID LateralSettings, PID HorizontalSettings);

     Drivetrain drivetrain;
     PID LateralSettings;
     PID HorizontalSettings;

     /**
     * @brief Allows for a user to imput dist speed and timout to drive to a point
     * 
     * @param dist the distance you want
     * @param speed The speed you want in volts
     * @param timeout the timeout/time it gives
     */
        void drive(float dist, float speed, float timeout);

    /**
     * @brief Allows for a user to imput dist speed and timout to drive to a point
     * 
     * @param dist the distance you want
     * @param speed The speed you want in volts
     * @param timeout the timeout/time it gives
     */
        void turn(float angle, float speed, float timeout);
};
};