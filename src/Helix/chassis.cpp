#include "Helix/helixApi.h" // IWYU pragma: keep
#include "pros/motor_group.hpp"
#include <cmath>
#include "main.h" // IWYU pragma: keep

namespace Helix {
    Chassis::Chassis(pros::MotorGroup& left, pros::MotorGroup& right, pros::Imu& imu, double wheel_diameter, double ratio) : leftSide(left), rightSide(right), imu(imu), wheelDiameter(wheel_diameter), gearRatio(ratio) , drivePID(0, 0, 0), turnPID(0,0,0) {}

    void Chassis::setDrivePID(double p, double i, double d, double max_i, double max_out, double tol) {
        drivePID = PID(p, i, d);
        drivePID.setLimits(max_i, max_out);
        drivePID.setTolerance(tol);
    }

    void Chassis::setTurnPID(double p, double i, double d, double max_i, double max_out, double tol) {
        turnPID = PID(p, i, d);
        turnPID.setLimits(max_i, max_out);
        turnPID.setTolerance(tol);
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

    void Chassis::turn(double targetDegrees) {
        imu.reset(); // Reset IMU heading to 0

        turnPID.reset();

        while (!turnPID.isSettled()) {
            double currentHeading = imu.get_heading();
            double power = turnPID.compute(targetDegrees, currentHeading);

            leftSide.move(-power);
            rightSide.move(power);

            pros::delay(10);
        }

        leftSide.brake();
        rightSide.brake();
    }
}