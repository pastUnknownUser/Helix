#include "Helix/helixApi.h" // IWYU pragma: keep
#include "pros/motor_group.hpp"
#include <cmath>

namespace Helix {
    Chassis::Chassis(pros::MotorGroup& left, pros::MotorGroup& right, double wheel_dia, double ratio) : leftSide(left), rightSide(right), wheelDiameter(wheel_dia), gearRatio(ratio) , drivePID(0, 0, 0) {}

    void Chassis::setDrivePID(double p, double i, double d, double max_i, double max_out, double tol) {
        drivePID = PID(p, i, d);
        drivePID.setLimits(max_i, max_out);
        drivePID.setTolerance(tol);
    }

    void Chassis::move(double targetInches) {
        double circumference = wheelDiameter * M_PI;
        double targetDegrees = (targetInches / circumference) * 360.0 * gearRatio;

        leftSide.set_zero_position(leftSide.get_position());
        rightSide.set_zero_position(rightSide.get_position());

        drivePID.reset();

        while (!drivePID.isSettled()) {
            double leftPos = leftSide.get_position();
            double rightPos = rightSide.get_position();
            double currentDegrees = (leftPos + rightPos) / 2.0;

            double power = drivePID.compute(targetDegrees, currentDegrees);

            leftSide.move(power);
            rightSide.move(power);

            pros::delay(10);
        }

        leftSide.brake();
        rightSide.brake();
    }
}