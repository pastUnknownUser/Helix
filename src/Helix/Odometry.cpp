#include "Helix/Odometry.hpp"
#include "pros/rtos.hpp"
#include <cmath>

namespace Helix {

Odometry::Odometry(const Config& cfg)
    : config(cfg),
      currentPose(0, 0, 0),
      totalDistance(0),
      prevLeftPos(0),
      prevRightPos(0),
      prevHorizontalPos(0),
      prevHeading(0),
      firstUpdate(true) {

    // Calculate encoder ticks per inch
    // Motor encoders: 360 degrees per rotation
    // ADI encoders: 360 ticks per rotation
    float wheelDiam = config.getWheelDiameter();
    ticksPerInch = 360.0f / (wheelDiam * 3.14159f);
    ticksPerInchH = ticksPerInch;  // Same for horizontal unless different wheel size

    // Initialize with current readings
    prevLeftPos = getLeftPosition();
    prevRightPos = getRightPosition();
    prevHorizontalPos = getHorizontalPosition();

    if (config.imu) {
        prevHeading = getHeading();
        currentPose.theta = prevHeading;
    }
}

void Odometry::update() {
    // Get current encoder readings
    float leftPos = getLeftPosition();
    float rightPos = getRightPosition();
    float horizontalPos = getHorizontalPosition();
    float heading = getHeading();

    if (firstUpdate) {
        prevLeftPos = leftPos;
        prevRightPos = rightPos;
        prevHorizontalPos = horizontalPos;
        prevHeading = heading;
        firstUpdate = false;
        return;
    }

    // Calculate deltas in inches
    float deltaLeft = (leftPos - prevLeftPos) / ticksPerInch;
    float deltaRight = (rightPos - prevRightPos) / ticksPerInch;
    float deltaHorizontal = (horizontalPos - prevHorizontalPos) / ticksPerInchH;

    // Average distance traveled (forward/backward)
    float deltaForward = (deltaLeft + deltaRight) / 2.0f;
    totalDistance += std::abs(deltaForward);

    // Calculate heading change from IMU (most accurate)
    float deltaHeading = angleDifference(heading, prevHeading);

    // Calculate local position change
    float deltaXLocal, deltaYLocal;

    if (std::abs(deltaHeading) < 0.001f) {
        // Straight line - no arc
        deltaXLocal = deltaForward;
        // Use horizontal encoder directly for Y, accounting for heading
        if (config.horizontalEncoder) {
            deltaYLocal = deltaHorizontal;
        } else {
            deltaYLocal = 0;
        }
    } else {
        // Arc approximation for forward movement
        float radius = deltaForward / (deltaHeading * 3.14159f / 180.0f);
        deltaXLocal = radius * std::sin(deltaHeading * 3.14159f / 180.0f);

        // Y has two components:
        // 1. Arc component from forward movement
        // 2. Direct horizontal encoder reading
        float arcY = radius * (1 - std::cos(deltaHeading * 3.14159f / 180.0f));

        // If we have a horizontal encoder, use it directly
        // Otherwise approximate from arc
        if (config.horizontalEncoder) {
            deltaYLocal = deltaHorizontal;
            // Account for the fact that horizontal wheel rotates during turn
            // due to being offset from center of rotation
            float offsetRotation = deltaHeading * 3.14159f / 180.0f * config.horizontalOffset;
            deltaYLocal -= offsetRotation;
        } else {
            deltaYLocal = arcY;
        }
    }

    // Convert to global coordinates
    float headingRad = currentPose.theta * 3.14159f / 180.0f;
    float deltaXGlobal = deltaXLocal * std::cos(headingRad) - deltaYLocal * std::sin(headingRad);
    float deltaYGlobal = deltaXLocal * std::sin(headingRad) + deltaYLocal * std::cos(headingRad);

    // Update pose
    currentPose.x += deltaXGlobal;
    currentPose.y += deltaYGlobal;
    currentPose.theta = heading;

    // Store for next iteration
    prevLeftPos = leftPos;
    prevRightPos = rightPos;
    prevHorizontalPos = horizontalPos;
    prevHeading = heading;
}

void Odometry::setPose(const Pose& pose) {
    currentPose = pose;
    prevHeading = pose.theta;

    // Reset motor encoders
    if (config.leftMotors) {
        config.leftMotors->tare_position();
    }
    if (config.rightMotors) {
        config.rightMotors->tare_position();
    }

    // Reset ADI encoders (set to 0)
    if (config.leftEncoder) {
        config.leftEncoder->reset();
    }
    if (config.rightEncoder) {
        config.rightEncoder->reset();
    }
    if (config.horizontalEncoder) {
        config.horizontalEncoder->reset();
    }

    prevLeftPos = getLeftPosition();
    prevRightPos = getRightPosition();
    prevHorizontalPos = getHorizontalPosition();
}

float Odometry::getLeftPosition() {
    // Prefer external encoder if available
    if (config.leftEncoder) {
        return static_cast<float>(config.leftEncoder->get_value());
    }

    // Fall back to motor encoders
    if (!config.leftMotors || config.leftMotors->size() == 0) {
        return 0;
    }

    double avg = 0;
    for (int i = 0; i < config.leftMotors->size(); i++) {
        avg += (*config.leftMotors)[i].get_position();
    }
    return static_cast<float>(avg / config.leftMotors->size());
}

float Odometry::getRightPosition() {
    // Prefer external encoder if available
    if (config.rightEncoder) {
        return static_cast<float>(config.rightEncoder->get_value());
    }

    // Fall back to motor encoders
    if (!config.rightMotors || config.rightMotors->size() == 0) {
        return 0;
    }

    double avg = 0;
    for (int i = 0; i < config.rightMotors->size(); i++) {
        avg += (*config.rightMotors)[i].get_position();
    }
    return static_cast<float>(avg / config.rightMotors->size());
}

float Odometry::getHorizontalPosition() {
    if (config.horizontalEncoder) {
        return static_cast<float>(config.horizontalEncoder->get_value());
    }
    return 0;  // No horizontal tracking available
}

float Odometry::getHeading() {
    if (!config.imu) {
        return 0;
    }
    return config.imu->get_heading();
}

float Odometry::angleDifference(float target, float current) {
    float diff = target - current;
    while (diff > 180) diff -= 360;
    while (diff < -180) diff += 360;
    return diff;
}

float Odometry::normalizeAngle(float angle) {
    while (angle >= 360) angle -= 360;
    while (angle < 0) angle += 360;
    return angle;
}

} // namespace Helix
