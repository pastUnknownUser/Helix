#include "main.h"
#include "Helix/auton_selector.hpp"

using namespace Helix;

AutonSelector::AutonSelector() {
  auton_count = 0;
  auton_page_current = 0;
  Autons = {};
}

AutonSelector::AutonSelector(std::vector<Auton> autons) {
  auton_count = autons.size();
  auton_page_current = 0;
  Autons = {};
  Autons.assign(autons.begin(), autons.end());
}

void AutonSelector::selected_auton_print() {
    if (auton_count == 0) return;

    #define COL_BG       0x00111111
    #define COL_ARROW_BG 0x00333333
    #define COL_WHITE    0x00FFFFFF
    #define COL_GREY     0x00888888
    #define COL_ACCENT   0x000077FF

    // Centre background
    pros::screen::set_pen(COL_BG);
    pros::screen::fill_rect(80, 0, 400, 240);

    // Left arrow panel
    pros::screen::set_pen(COL_ARROW_BG);
    pros::screen::fill_rect(0, 0, 80, 240);
    for (int i = 0; i < 20; i++) {
        pros::screen::set_pen(COL_WHITE);
        pros::screen::draw_line(30 + i, 120 - i, 30 + i, 120 + i);
    }

    // Right arrow panel
    pros::screen::set_pen(COL_ARROW_BG);
    pros::screen::fill_rect(400, 0, 480, 240);
    for (int i = 0; i < 20; i++) {
        pros::screen::set_pen(COL_WHITE);
        pros::screen::draw_line(450 - i, 120 - i, 450 - i, 120 + i);
    }

    // Accent bars
    pros::screen::set_pen(COL_ACCENT);
    pros::screen::fill_rect(80, 0, 400, 6);
    pros::screen::fill_rect(80, 234, 400, 240);

    // Index text
    pros::screen::set_pen(COL_BG);
    pros::screen::fill_rect(81, 40, 399, 85);
    pros::screen::set_pen(COL_GREY);
    pros::screen::print(pros::E_TEXT_MEDIUM, 195, 52,
        "%d / %d", auton_page_current + 1, auton_count);

    // Auton name
    pros::screen::set_pen(COL_BG);
    pros::screen::fill_rect(81, 86, 399, 160);
    pros::screen::set_pen(COL_WHITE);
    pros::screen::print(pros::E_TEXT_LARGE, 95, 105,
        "%s", Autons[auton_page_current].Name.c_str());
}

void AutonSelector::selected_auton_call() {
    if (auton_count == 0) return;
    Autons[auton_page_current].auton_call(); // just use auton_page_current directly
}

void AutonSelector::autons_add(std::vector<Auton> autons) {
  auton_count += autons.size();
  auton_page_current = 0;
  Autons.assign(autons.begin(), autons.end());
}