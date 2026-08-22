#include "main.h"
#include "external_config.hpp" // IWYU pragma: keep
#include "Helix/helixApi.h" // IWYU pragma: keep
#include "pros/misc.h"
#include <cstdlib>

Helix::AutonSelector auton_selector;

void initialize() {
    pros::delay(500);
    initializeChassis();

    auton_selector.autons_add({
        Helix::Auton("Red Left", moveStraight, "Drive forward 10 inches", true, 0),
        Helix::Auton("Red Right", turn90Degrees, "Test a 90-degree in-place turn", true, 0),
        Helix::Auton("Blue Left", blueLeft, "Drive forward 10 inches", false, 0),
        Helix::Auton("Skills", skills, "Drive 10 inches, then turn 90 degrees", false, 0),
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
    bool rumble_latched = false;

    while (true) {
        int dir  = master.get_analog(ANALOG_LEFT_Y);
        int turn = master.get_analog(ANALOG_RIGHT_X);

        // Small stick noise is common on V5 controllers and makes the robot
        // creep at rest, so keep a quiet center band.
        if (std::abs(dir) < 5) dir = 0;
        if (std::abs(turn) < 5) turn = 0;

        leftSide.move(dir - turn);
        rightSide.move(dir + turn);

        bool intake_button = master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        if (intake_button && !rumble_latched) master.rumble(".");
        rumble_latched = intake_button;

        pros::delay(20);
    }
}
