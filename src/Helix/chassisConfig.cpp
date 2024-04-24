#include "Helix/api.hpp"
#include "pros/apix.h"
#include "api.h"

Helix::PID::PID(float kP, float kI, float kD, int integralTerm, int previousError) :
kP(kP),
kI(kI),
kD(kD),
integralTerm(integralTerm),
previousError(previousError)
{}

Helix::Drivetrain::Drivetrain(pros::MotorGroup* LeftSide, pros::MotorGroup* RightSide, float Rpm, float WheelDiameter) :

       Leftside(Leftside),
       RightSide(RightSide),
       Rpm(Rpm),
       WheelDiameter(WheelDiameter)
{}

Helix::Chassis::Chassis(Drivetrain drivetrain, PID LateralSettings, PID HorizontalSettings) :

        drivetrain(drivetrain),
        LateralSettings(LateralSettings),
        HorizontalSettings(HorizontalSettings)
{}

void Helix::Chassis::drive(float dist, float speed, float timeout) {
    double error = dist - speed;
    LateralSettings.integralTerm += error * timeout; // Assuming integralTerm is part of LateralSettings
    double derivativeTerm = (error - LateralSettings.previousError) / timeout; // Assuming previousError is part of LateralSettings
    double output = LateralSettings.kP * error + LateralSettings.kI * LateralSettings.integralTerm + LateralSettings.kD * derivativeTerm; // Assuming kP, kI, kD are part of LateralSettings
    LateralSettings.previousError = error; // Assuming previousError is part of LateralSettings

    //Moves each side of the drive with the velocity of the PID output
    double left_velocity = drivetrain.Rpm + output;
    double right_velocity = drivetrain.Rpm - output;
    
    drivetrain.Leftside->move_velocity(left_velocity);
    drivetrain.RightSide->move_velocity(right_velocity);
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

    drivetrain.Leftside->move_voltage(left_velocity);
    drivetrain.RightSide->move_voltage(right_velocity);

}