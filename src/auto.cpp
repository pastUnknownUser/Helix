#include "Helix/chassis.h"
#include "main.h" // IWYU pragma: keep
#include "auto.hpp"

extern Helix::Chassis chassis;

namespace {
    // Hardware values from the incoming robot configuration.
    constexpr double kWheelDiameterIn = 3.25;
    constexpr double kExternalGearRatio = 1.33333;

    void configureTestDrive() {
        // Outputs are millivolts. These are conservative starting gains for
        // an initial drivetrain test, not a final competition tune.
        chassis.setDrivePID(10.0, 0.0, 20.0, 0.0, 9000.0, 5.0);
        chassis.setTurnPID(75.0, 0.0, 20.0, 0.0, 9000.0, 1.0);
    }
}

pros::MotorGroup leftSide({-8, -19, -20}, pros::MotorGearset::blue);
pros::MotorGroup rightSide({18, 4, 1}, pros::MotorGearset::blue);
pros::Imu imu(14);
Helix::Chassis chassis(leftSide, rightSide, imu, kWheelDiameterIn, kExternalGearRatio);

void initializeChassis() {
    imu.reset(true);
}

void moveStraight() {
    configureTestDrive();
    chassis.move(10);
}

void turn90Degrees() {
    configureTestDrive();
    chassis.turn(90);
}

void blueLeft() {
    configureTestDrive();
    chassis.move(10);
}

void skills() {
    configureTestDrive();
    chassis.move(10);
    chassis.turn(90);
}
