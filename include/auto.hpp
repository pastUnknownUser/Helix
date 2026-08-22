#pragma once

#include "pros/motor_group.hpp"

extern pros::MotorGroup leftSide;
extern pros::MotorGroup rightSide;

void moveStraight();
void turn90Degrees();
void blueLeft();
void skills();
void initializeChassis();
