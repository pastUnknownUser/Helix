#pragma once
#include <vector>
#include "Helix/auton.hpp"
#include "liblvgl/lvgl.h"

namespace Helix {

enum class SelectorView { LIST, DETAIL };

class AutonSelector {
public:
    std::vector<Auton> Autons;
    int auton_page_current;
    int auton_count;
    SelectorView current_view;

    lv_obj_t* screen;
    lv_obj_t* list_panel;
    lv_obj_t* detail_panel;

    AutonSelector();
    AutonSelector(std::vector<Auton> autons);
    void selected_auton_call();
    void selected_auton_print();  // ← add this back
    void init_ui();
    void show_list();
    void show_detail(int index);
    void autons_add(std::vector<Auton> autons);
};
}