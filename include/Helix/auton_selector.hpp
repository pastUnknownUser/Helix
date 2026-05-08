#pragma once
#include <tuple>

#include "Helix/auton.hpp"

using namespace std;

namespace Helix {
    class AutonSelector {
        public:
            std::vector<Auton> Autons;
            int auton_page_current;
            int auton_count;
            int last_auton_page_current;
            AutonSelector();
            AutonSelector(std::vector<Auton> autons);
            void selected_auton_call();
            void selected_auton_print();
            void autons_add(std::vector<Auton> autons);
    };
}  // namespace Helix