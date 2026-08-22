#include "Helix/helixApi.h" // IWYU pragma: keep
#include <utility>

namespace Helix {

Auton::Auton() {
    Name = "";
    Description = "";
    IsRed = true;
    ExpectedPoints = 0;
    auton_call = nullptr;
}

Auton::Auton(std::string name, std::function<void()> call,
             std::string description, bool is_red, int expected_points) {
    Name = std::move(name);
    Description = std::move(description);
    IsRed = is_red;
    ExpectedPoints = expected_points;
    auton_call = std::move(call);
}

}
