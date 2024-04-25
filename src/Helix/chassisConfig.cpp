#include "Helix/api.hpp"
#include "pros/apix.h"
#include "api.h"
#include <stdio.h>
#include <cstdio>


Helix::PID::PID(float kP, float kI, float kD, int integralTerm, int previousError, int antiWindup) :
kP(kP),
kI(kI),
kD(kD),
integralTerm(integralTerm),
previousError(previousError),
antiWindup(antiWindup)
{}

Helix::Drivetrain::Drivetrain(pros::MotorGroup* LeftSide, pros::MotorGroup* RightSide, float Rpm, float WheelDiameter) :

       LeftSide(LeftSide),
       RightSide(RightSide),
       Rpm(Rpm),
       WheelDiameter(WheelDiameter)
{}

Helix::Sensors::Sensors(pros::IMU* Imu) :
        Imu(Imu)
{}

Helix::Chassis::Chassis(Drivetrain drivetrain, PID LateralSettings, PID HorizontalSettings, Sensors sensors) :

        drivetrain(drivetrain),
        LateralSettings(LateralSettings),
        HorizontalSettings(HorizontalSettings),
        sensors(sensors)
{}

// Stop motors using brake mode
void Helix::Chassis::stopDrive() {
    // Apply brake mode to both sides of the drivetrain
    LeftSideDrive.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
    RightSideDrive.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);

}

void Helix::Chassis::drive(float dist, float speed, float timeout) {

    // Reset encoder values
    LeftSideDrive.tare_position();
    RightSideDrive.tare_position();

    // Initialize loop variables
    bool targetReached = false;
    double maxOutput = 127.0; // Maximum output voltage
    double TickToInch = drivetrain.Rpm / (drivetrain.WheelDiameter * M_PI);
    double Inches = dist * TickToInch + 1;

    while (!targetReached) {

        // Calculate actual distance traveled
        double actDistance = (leftFront.get_position() + rightFront.get_position()) / 2.0;
        
        // Calculate error and update integral term
        double error = Inches - actDistance;
        LateralSettings.integralTerm += error * timeout;
        
        // Apply anti-windup to integral term
        if (LateralSettings.integralTerm > (maxOutput * LateralSettings.antiWindup)) {
            LateralSettings.integralTerm = maxOutput * LateralSettings.antiWindup;
        } else if (LateralSettings.integralTerm < -maxOutput * LateralSettings.antiWindup) {
            LateralSettings.integralTerm = -maxOutput * LateralSettings.antiWindup;
        }

        // Calculate derivative term
        double derivativeTerm = (error - LateralSettings.previousError) / timeout;
        
        // Calculate PID output
        double output = LateralSettings.kP * error + LateralSettings.kI * LateralSettings.integralTerm + LateralSettings.kD * derivativeTerm;
        
        // Update previous error for next iteration
        LateralSettings.previousError = error;

        // Limit output to prevent excessive motor speed
        if (output > maxOutput) {
            output = maxOutput;
        } else if (output < -maxOutput) {
            output = -maxOutput;
        }

         // Apply PID output to left and right motor velocities
         double left_velocity = speed + output;
         double right_velocity = speed + output; 

         drivetrain.LeftSide->move_velocity(left_velocity);
         drivetrain.RightSide->move_velocity(right_velocity);

         pros::delay(10);
         // Check if target distance is reached
         if (Inches <= actDistance) {

         // Stop motors when target reached
         stopDrive();
         targetReached = true;
    }
    pros::delay(100);
    }
}

void Helix::Chassis::turn(float angle, float speed, float timeout) {
    double error = angle - speed;
    HorizontalSettings.integralTerm += error * timeout;
    double derivativeTerm = (error - HorizontalSettings.previousError) / timeout;

    double output = HorizontalSettings.kP * error + HorizontalSettings.kI * HorizontalSettings.integralTerm + HorizontalSettings.kD * derivativeTerm;
    HorizontalSettings.previousError = error;

    //Moves each side of the drive with the velocity of the PID output
    double left_velocity = drivetrain.Rpm + output;
    double right_velocity = drivetrain.Rpm - output;

    drivetrain.LeftSide->move_voltage(left_velocity);
    drivetrain.RightSide->move_voltage(right_velocity);

}