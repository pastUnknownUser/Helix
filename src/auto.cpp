#pragma once
#include "Helix/chassis.h"
#include "main.h" // IWYU pragma: keep
#include "Helix/helixApi.h" // IWYU pragma: keep
#include "pros/misc.h" // IWYU pragma: keep
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"

// Autos for competition \\

pros::MotorGroup leftSide({1, -2, 3});
pros::MotorGroup rightSide({-4, 5, -6});

Helix::Chassis chassis(leftSide, rightSide, "wheel diameter", "gear ratio");

void moveStraight(){
    chassis.setDrivePID(.1, .001, .3, 1000, 10, 5);
    chassis.move(24);
    //chassis.dosomethingidfk
    //master.rumble("...");
}

void turn90Degrees(){
    //chassis.turnsomethingidfk
}

void example3(){
    //Do something here
}

void example4(){
    //Do something here
}