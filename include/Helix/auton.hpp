#pragma once
#include <functional>
#include <iostream>
#include <vector>

namespace Helix {
class Auton {
public:
    Auton();
    Auton(std::string name,
          std::function<void()> call,
          std::string description,
          bool is_red,
          int expected_points);  // ← add this

    std::string Name;
    std::string Description;
    bool IsRed;
    int ExpectedPoints;          // ← add this
    std::function<void()> auton_call;
};
}