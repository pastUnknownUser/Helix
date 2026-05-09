#include "helixApi.h"

namespace Helix {

Auton::Auton() {
    Name = "";
    Description = "";
    IsRed = true;
    auton_call = nullptr;
}

Auton::Auton(std::string name, std::function<void()> call,
             std::string description, bool is_red, int expected_points) {
    Name = name;
    Description = description;
    IsRed = is_red;
    ExpectedPoints = expected_points;
    auton_call = call;
}

}