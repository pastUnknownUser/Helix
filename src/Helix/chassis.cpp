#include "Helix/helixApi.h" // IWYU pragma: keep
#include "pros/motor_group.hpp"
#include <algorithm>
#include <cmath>

namespace Helix {
    namespace {
        constexpr int kLoopPeriodMs = 10;
        constexpr int kSettleLoops = 5;
        constexpr int kMoveTimeoutMs = 4000;
        constexpr int kTurnTimeoutMs = 3000;

        int clampVoltage(double voltage) {
            return static_cast<int>(std::clamp(voltage, -12000.0, 12000.0));
        }

        double wrapHeadingError(double error) {
            while (error > 180.0) error -= 360.0;
            while (error < -180.0) error += 360.0;
            return error;
        }
    }

    Chassis::Chassis(pros::MotorGroup& left, pros::MotorGroup& right,
                     pros::Imu& inertial, double wheel_dia, double ratio)
        : leftSide(left), rightSide(right), imu(inertial),
          drivePID(0, 0, 0), turnPID(0, 0, 0),
          wheelDiameter(wheel_dia), gearRatio(ratio) {}

    void Chassis::setDrivePID(double p, double i, double d, double max_i,
                              double max_out, double tol) {
        drivePID = PID(p, i, d);
        drivePID.setLimits(max_i, max_out);
        drivePID.setTolerance(tol);
    }

    void Chassis::setTurnPID(double p, double i, double d, double max_i,
                             double max_out, double tol) {
        turnPID = PID(p, i, d);
        turnPID.setLimits(max_i, max_out);
        turnPID.setTolerance(tol);
    }

    void Chassis::move(double targetInches) {
        if (wheelDiameter <= 0 || gearRatio <= 0) return;

        double circumference = wheelDiameter * M_PI;
        double targetDegrees = (targetInches / circumference) * 360.0 * gearRatio;

        leftSide.tare_position();
        rightSide.tare_position();
        drivePID.reset();

        int settledLoops = 0;
        for (int elapsed = 0; elapsed < kMoveTimeoutMs; elapsed += kLoopPeriodMs) {
            double leftPos = leftSide.get_position();
            double rightPos = rightSide.get_position();
            double currentDegrees = (leftPos + rightPos) / 2.0;
            double power = drivePID.compute(targetDegrees, currentDegrees);

            leftSide.move_voltage(clampVoltage(power));
            rightSide.move_voltage(clampVoltage(power));

            if (drivePID.isSettled()) {
                ++settledLoops;
                if (settledLoops >= kSettleLoops) break;
            } else {
                settledLoops = 0;
            }

            pros::delay(kLoopPeriodMs);
        }

        leftSide.brake();
        rightSide.brake();
    }

    void Chassis::turn(double targetDegrees) {
        if (imu.is_calibrating()) return;

        imu.tare_heading();
        pros::delay(kLoopPeriodMs);
        turnPID.reset();

        int settledLoops = 0;
        for (int elapsed = 0; elapsed < kTurnTimeoutMs; elapsed += kLoopPeriodMs) {
            double headingError = wrapHeadingError(targetDegrees - imu.get_heading());
            double power = turnPID.compute(headingError, 0.0);

            leftSide.move_voltage(clampVoltage(-power));
            rightSide.move_voltage(clampVoltage(power));

            if (turnPID.isSettled()) {
                ++settledLoops;
                if (settledLoops >= kSettleLoops) break;
            } else {
                settledLoops = 0;
            }

            pros::delay(kLoopPeriodMs);
        }

        leftSide.brake();
        rightSide.brake();
    }
}
