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
      prevHeading(0),
      firstUpdate(true) {

    // Calculate encoder ticks per inch
    // 360 degrees per rotation, wheel circumference = π * diameter
    // RPM doesn't matter for encoders - they report in degrees
    ticksPerInch = 360.0f / (config.wheelDiameter * 3.14159f);

    // Initialize with current readings
    if (config.leftMotors) {
        prevLeftPos = getLeftPosition();
    }
    if (config.rightMotors) {
        prevRightPos = getRightPosition();
    }
    if (config.imu) {
        prevHeading = getHeading();
        currentPose.theta = prevHeading;
    }
}

void Odometry::update() {
    // Get current encoder readings
    float leftPos = getLeftPosition();
    float rightPos = getRightPosition();
    float heading = getHeading();

    if (firstUpdate) {
        prevLeftPos = leftPos;
        prevRightPos = rightPos;
        prevHeading = heading;
        firstUpdate = false;
        return;
    }

    // Calculate deltas
    float deltaLeft = (leftPos - prevLeftPos) / ticksPerInch;
    float deltaRight = (rightPos - prevRightPos) / ticksPerInch;

    // Average distance traveled
    float deltaDistance = (deltaLeft + deltaRight) / 2.0f;
    totalDistance += std::abs(deltaDistance);

    // Calculate heading change
    float deltaHeading = angleDifference(heading, prevHeading);

    // Calculate position change in local frame
    float deltaXLocal = 0;
    float deltaYLocal = 0;

    if (std::abs(deltaHeading) < 0.001f) {
        // Straight line approximation (no turn)
        deltaXLocal = deltaDistance;
        deltaYLocal = 0;
    } else {
        // Arc approximation
        float radius = deltaDistance / (deltaHeading * 3.14159f / 180.0f);
        deltaXLocal = radius * std::sin(deltaHeading * 3.14159f / 180.0f);
        deltaYLocal = radius * (1 - std::cos(deltaHeading * 3.14159f / 180.0f));
    }

    // Convert to global coordinates
    float headingRad = currentPose.theta * 3.14159f / 180.0f;
    float deltaXGlobal = deltaXLocal * std::cos(headingRad) - deltaYLocal * std::sin(headingRad);
    float deltaYGlobal = deltaXLocal * std::sin(headingRad) + deltaYLocal * std::cos(headingRad);

    // Update pose
    currentPose.x += deltaXGlobal;
    currentPose.y += deltaYGlobal;
    currentPose.theta = heading;  // Use IMU heading directly

    // Store for next iteration
    prevLeftPos = leftPos;
    prevRightPos = rightPos;
    prevHeading = heading;
}

void Odometry::setPose(const Pose& pose) {
    currentPose = pose;
    prevHeading = pose.theta;

    // Reset encoders to new reference
    if (config.leftMotors) {
        config.leftMotors->tare_position();
        prevLeftPos = 0;
    }
    if (config.rightMotors) {
        config.rightMotors->tare_position();
        prevRightPos = 0;
    }
}

float Odometry::getLeftPosition() {
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
    if (!config.rightMotors || config.rightMotors->size() == 0) {
        return 0;
    }

    double avg = 0;
    for (int i = 0; i < config.rightMotors->size(); i++) {
        avg += (*config.rightMotors)[i].get_position();
    }
    return static_cast<float>(avg / config.rightMotors->size());
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
