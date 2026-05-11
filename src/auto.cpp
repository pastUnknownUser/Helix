#pragma once
#include "Helix/chassis.h"
#include "main.h" // IWYU pragma: keep
#include "Helix/helixApi.h" // IWYU pragma: keep
#include "pros/misc.h" // IWYU pragma: keep
#include "pros/motor_group.hpp"
#include "pros/motors.hpp" //IWYU pragma: keep

// Autos for competition \\

inline pros::MotorGroup left_motors({-8, -19, -20}, pros::MotorGearset::blue); // left motors on ports 3, -13, -11
inline pros::MotorGroup right_motors({18, 4 ,1}, pros::MotorGearset::blue); // right motors on ports -7, 18, 19
inline pros::Imu imu(14); // imu on port 14
Helix::Chassis chassis(
    left_motors, 
    right_motors, 
    imu, 
    3.25, 
    1.33333);

void moveStraight(){
    chassis.setDrivePID(
        10, 
        .001, 
        5, 
        1200, 
        10, 
        5);
        chassis.setTurnPID(
        10, 
        .001, 
        5, 
        1200, 
        10, 
        5);
    chassis.move(10);
    chassis.turn(90);
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