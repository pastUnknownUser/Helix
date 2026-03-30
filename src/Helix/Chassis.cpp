#include "Helix/Chassis.hpp"
#include "Helix/Odometry.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <vector>

// Define M_PI if not available (some platforms require _USE_MATH_DEFINES)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace Helix {

namespace {
    // Helper to average a vector of positions
    float averagePosition(const std::vector<double>& positions) {
        if (positions.empty()) return 0.0f;
        double sum = 0;
        for (double pos : positions) {
            sum += pos;
        }
        return static_cast<float>(sum / positions.size());
    }
}

// Drivetrain implementation
float Drivetrain::getAveragePosition() const {
    if (!leftMotors || !rightMotors) return 0.0f;

    float leftAvg = averagePosition(leftMotors->get_positions());
    float rightAvg = averagePosition(rightMotors->get_positions());

    return (leftAvg + rightAvg) / 2.0f;
}

float Drivetrain::getPositionDifference() const {
    if (!leftMotors || !rightMotors) return 0.0f;

    float leftAvg = averagePosition(leftMotors->get_positions());
    float rightAvg = averagePosition(rightMotors->get_positions());

    // Positive = turned right (left traveled more than right)
    return leftAvg - rightAvg;
}

void Drivetrain::tarePosition() const {
    if (leftMotors) leftMotors->tare_position();
    if (rightMotors) rightMotors->tare_position();
}

// Chassis implementation
Chassis::Chassis(const Config& config)
    : drivetrain_(config.drivetrain),
      lateralPID_(config.lateralPID.kP, config.lateralPID.kI, config.lateralPID.kD),
      turnPID_(config.turnPID.kP, config.turnPID.kI, config.turnPID.kD),
      imu_(config.imu),
      maxLateralSpeed_(config.maxLateralSpeed),
      maxTurnSpeed_(config.maxTurnSpeed),
      defaultTimeout_(config.defaultTimeout),
      odometry_(nullptr) {}

bool Chassis::waitForSettle(PIDController& pid, int timeout) {
    int elapsed = 0;
    const int dt = 20; // 20ms delay

    while (!pid.isSettled() && elapsed < timeout) {
        // Update odometry if available to prevent stale data
        if (odometry_) {
            odometry_->update();
        }
        pros::delay(dt);
        elapsed += dt;
    }

    return pid.isSettled();
}

bool Chassis::drive(float distance, float maxSpeed, int timeout) {
    // Use defaults if not specified
    if (maxSpeed < 0) maxSpeed = maxLateralSpeed_;
    if (timeout < 0) timeout = defaultTimeout_;

    // Reset PID and encoders
    lateralPID_.reset();
    drivetrain_.tarePosition();

    // Set output limits for this movement
    lateralPID_.setOutputLimits(-maxSpeed, maxSpeed);

    // Calculate target in encoder ticks
    float targetTicks = distance * drivetrain_.ticksPerInch();

    // PID loop
    int elapsed = 0;
    const int dt = 20;

    while (!lateralPID_.isSettled() && elapsed < timeout) {
        float currentPos = drivetrain_.getAveragePosition();
        float output = lateralPID_.compute(targetTicks, currentPos);

        // Apply to both sides (with null check)
        if (drivetrain_.leftMotors && drivetrain_.rightMotors) {
            drivetrain_.leftMotors->move(output);
            drivetrain_.rightMotors->move(output);
        }

        pros::delay(dt);
        elapsed += dt;
    }

    stop();
    return lateralPID_.isSettled();
}

bool Chassis::turn(float angle, float maxSpeed, int timeout) {
    // Use defaults if not specified
    if (maxSpeed < 0) maxSpeed = maxTurnSpeed_;
    if (timeout < 0) timeout = defaultTimeout_;

    // Reset PID and encoders
    turnPID_.reset();
    drivetrain_.tarePosition();

    // Set output limits
    turnPID_.setOutputLimits(-maxSpeed, maxSpeed);

    // Calculate approximate tick difference for the turn
    // For a differential drive, turning 360 degrees means wheels travel a circle
    // with diameter equal to track width (assumed to be 12 inches if not specified)
    // This is a rough approximation - IMU-based turns are more accurate
    float trackWidth = 12.0f; // inches, adjust for your robot
    float wheelCircumference = drivetrain_.wheelDiameter * static_cast<float>(M_PI);
    float rotationCircumference = trackWidth * static_cast<float>(M_PI);
    float targetTickDiff = (angle / 360.0f) * rotationCircumference * drivetrain_.ticksPerInch();

    // PID loop
    int elapsed = 0;
    const int dt = 20;

    while (!turnPID_.isSettled() && elapsed < timeout) {
        float currentDiff = drivetrain_.getPositionDifference();
        float output = turnPID_.compute(targetTickDiff, currentDiff);

        // Apply opposite to each side (positive = turn right) with null check
        if (drivetrain_.leftMotors && drivetrain_.rightMotors) {
            drivetrain_.leftMotors->move(output);
            drivetrain_.rightMotors->move(-output);
        }

        pros::delay(dt);
        elapsed += dt;
    }

    stop();
    return turnPID_.isSettled();
}

bool Chassis::turnTo(float heading, float maxSpeed, int timeout) {
    if (!imu_) {
        // Warn and fall back to encoder-based turn if no IMU
        // In a real environment, you might want to log this
        return turn(heading, maxSpeed, timeout);
    }

    // Use defaults if not specified
    if (maxSpeed < 0) maxSpeed = maxTurnSpeed_;
    if (timeout < 0) timeout = defaultTimeout_;

    // Reset PID
    turnPID_.reset();

    // Set output limits
    turnPID_.setOutputLimits(-maxSpeed, maxSpeed);

    // Get current heading
    float currentHeading = imu_->get_heading();

    // Calculate shortest path
    float error = heading - currentHeading;
    while (error > 180) error -= 360;
    while (error < -180) error += 360;

    float targetHeading = currentHeading + error;

    // PID loop
    int elapsed = 0;
    const int dt = 20;

    while (!turnPID_.isSettled() && elapsed < timeout) {
        currentHeading = imu_->get_heading();

        // Calculate error with wraparound handling
        error = targetHeading - currentHeading;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        // Compute output - error is the direct setpoint
        float output = turnPID_.compute(error, 0);

        // Apply opposite to each side with null check
        if (drivetrain_.leftMotors && drivetrain_.rightMotors) {
            drivetrain_.leftMotors->move(output);
            drivetrain_.rightMotors->move(-output);
        }

        pros::delay(dt);
        elapsed += dt;
    }

    stop();
    return turnPID_.isSettled();
}

void Chassis::stop(pros::motor_brake_mode_e_t brakeMode) {
    if (drivetrain_.leftMotors) {
        drivetrain_.leftMotors->move(0);
        drivetrain_.leftMotors->set_brake_modes(brakeMode);
    }
    if (drivetrain_.rightMotors) {
        drivetrain_.rightMotors->move(0);
        drivetrain_.rightMotors->set_brake_modes(brakeMode);
    }
}

void Chassis::tank(int leftVoltage, int rightVoltage) {
    // Clamp to valid range
    if (leftVoltage > 127) leftVoltage = 127;
    if (leftVoltage < -127) leftVoltage = -127;
    if (rightVoltage > 127) rightVoltage = 127;
    if (rightVoltage < -127) rightVoltage = -127;

    if (drivetrain_.leftMotors) drivetrain_.leftMotors->move(leftVoltage);
    if (drivetrain_.rightMotors) drivetrain_.rightMotors->move(rightVoltage);
}

void Chassis::arcade(int forward, int turn) {
    // Clamp inputs
    if (forward > 127) forward = 127;
    if (forward < -127) forward = -127;
    if (turn > 127) turn = 127;
    if (turn < -127) turn = -127;

    // Calculate motor values
    int left = forward + turn;
    int right = forward - turn;

    // Normalize if values exceed 127
    int maxVal = left > right ? left : right;
    int minVal = left < right ? left : right;
    int absMax = maxVal > -minVal ? maxVal : -minVal;

    if (absMax > 127) {
        float scale = 127.0f / absMax;
        left = (int)(left * scale);
        right = (int)(right * scale);
    }

    tank(left, right);
}

bool Chassis::driveToPoint(float x, float y, float maxSpeed, int timeout) {
    if (!odometry_) {
        // Fall back to regular drive if no odometry
        return false;
    }

    // Use defaults if not specified
    if (maxSpeed < 0) maxSpeed = maxLateralSpeed_;
    if (timeout < 0) timeout = defaultTimeout_;

    // Reset PID
    lateralPID_.reset();
    lateralPID_.setOutputLimits(-maxSpeed, maxSpeed);

    // Create a PID for heading correction during movement
    PIDController headingPID(navConfig_.headingPID.kP, navConfig_.headingPID.kI, navConfig_.headingPID.kD);
    headingPID.reset();
    headingPID.setOutputLimits(-navConfig_.headingCorrectionLimit, navConfig_.headingCorrectionLimit);

    Pose target(x, y, 0);
    int elapsed = 0;
    const int dt = 20;

    while (elapsed < timeout) {
        Pose current = odometry_->getPose();

        // Calculate distance to target
        float distance = current.distanceTo(target);

        // Check if we've arrived (using PID settled check to avoid overshoot)
        if (distance < navConfig_.arrivalThreshold && lateralPID_.isSettled()) {
            break;
        }

        // Calculate angle to target
        float angleToTarget = current.angleTo(target);

        // Calculate heading error
        float headingError = Odometry::angleDifference(angleToTarget, current.theta);

        // Compute outputs - target is 0 (we want error to be 0)
        float distanceOutput = lateralPID_.compute(-distance, 0);  // Drive toward target
        float headingOutput = headingPID.compute(-headingError, 0);  // Turn toward target

        // Combine: distance drives forward/back, heading steers
        int leftSpeed = (int)(distanceOutput - headingOutput);
        int rightSpeed = (int)(distanceOutput + headingOutput);

        // Clamp to motor range
        if (leftSpeed > maxSpeed) leftSpeed = (int)maxSpeed;
        if (leftSpeed < -maxSpeed) leftSpeed = (int)-maxSpeed;
        if (rightSpeed > maxSpeed) rightSpeed = (int)maxSpeed;
        if (rightSpeed < -maxSpeed) rightSpeed = (int)-maxSpeed;

        // Apply with null check
        if (drivetrain_.leftMotors && drivetrain_.rightMotors) {
            tank(leftSpeed, rightSpeed);
        }

        pros::delay(dt);
        elapsed += dt;
    }

    stop();
    return (elapsed < timeout);
}

bool Chassis::turnToPoint(float x, float y, float maxSpeed, int timeout) {
    if (!odometry_) {
        // Fall back to regular turn if no odometry
        return false;
    }

    // Use defaults if not specified
    if (maxSpeed < 0) maxSpeed = maxTurnSpeed_;
    if (timeout < 0) timeout = defaultTimeout_;

    // Reset PID
    turnPID_.reset();
    turnPID_.setOutputLimits(-maxSpeed, maxSpeed);
    turnPID_.setTolerance(navConfig_.turnThreshold, navConfig_.turnSettleSamples);

    Pose target(x, y, 0);
    int elapsed = 0;
    const int dt = 20;

    while (!turnPID_.isSettled() && elapsed < timeout) {
        Pose current = odometry_->getPose();

        // Calculate angle to target
        float angleToTarget = current.angleTo(target);

        // Calculate error
        float error = Odometry::angleDifference(angleToTarget, current.theta);

        // Compute output - error is the direct setpoint
        float output = turnPID_.compute(error, 0);

        // Apply to motors (opposite sides for turning) with null check
        if (drivetrain_.leftMotors && drivetrain_.rightMotors) {
            drivetrain_.leftMotors->move(output);
            drivetrain_.rightMotors->move(-output);
        }

        pros::delay(dt);
        elapsed += dt;
    }

    stop();
    return turnPID_.isSettled();
}

Pose Chassis::getPose() const {
    if (odometry_) {
        return odometry_->getPose();
    }
    return Pose(0, 0, 0);
}

void Chassis::setPose(const Pose& pose) {
    if (odometry_) {
        odometry_->setPose(pose);
    }
}

void Chassis::setOdometry(Odometry* odom) {
    odometry_ = odom;
}

} // namespace Helix
