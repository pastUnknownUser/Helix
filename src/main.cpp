#include "main.h"
#include "Helix/auton_selector.hpp"
#include "helixApi.h"

Helix::AutonSelector auton_selector;
pros::Task* touch_task = nullptr;

void initialize() {
    pros::delay(500);

    auton_selector.autons_add({
    Helix::Auton("Red Left",  moveStraight,  "Score stake + 4 rings, AWP eligible", true,  18),
    Helix::Auton("Red Right", turn90Degrees, "Safe side, mobile goal + 2 rings",    true,  12),
    Helix::Auton("Blue Left", example3,      "AWP + stake climb",                   false, 15),
});

    auton_selector.init_ui();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    auton_selector.selected_auton_call();
}

void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    pros::MotorGroup left_mg({1, -2, 3});
    pros::MotorGroup right_mg({-4, 5, -6});

    while (true) {
        int dir  = master.get_analog(ANALOG_LEFT_Y);
        int turn = master.get_analog(ANALOG_RIGHT_X);
        left_mg.move(dir - turn);
        right_mg.move(dir + turn);
        pros::delay(20);
    }
}