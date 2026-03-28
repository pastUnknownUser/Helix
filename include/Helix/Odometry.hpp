#pragma once

#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include <cmath>

namespace Helix {

/**
 * @brief Robot pose (position and heading)
 *
 * Represents the robot's position on the field in inches
 * and heading in degrees (0-360, CW positive from positive X)
 */
struct Pose {
    float x;        // X position in inches (field coordinates)
    float y;        // Y position in inches (field coordinates)
    float theta;    // Heading in degrees (0 = +X, 90 = +Y, CW positive)

    /**
     * @brief Default constructor - initializes to origin
     */
    Pose() : x(0), y(0), theta(0) {}

    /**
     * @brief Construct from coordinates
     */
    Pose(float xPos, float yPos, float heading) : x(xPos), y(yPos), theta(heading) {}

    /**
     * @brief Get distance to another pose
     */
    float distanceTo(const Pose& other) const {
        float dx = x - other.x;
        float dy = y - other.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    /**
     * @brief Get angle to another pose in degrees
     */
    float angleTo(const Pose& other) const {
        float dx = other.x - x;
        float dy = other.y - y;
        float angle = std::atan2(dy, dx) * 180.0f / 3.14159f;
        return angle;
    }
};

/**
 * @brief Odometry system for tracking robot position
 *
 * Uses wheel encoders and IMU to track the robot's (x, y) position
 * and heading on the field. Works with:
 * - Drive wheels only (simpler, less accurate during turns)
 * - Dedicated tracking wheels (more accurate, recommended)
 *
 * Example:
 * @code
 * // Setup with drive wheels only
 * Helix::Odometry odom(&leftMotors, &rightMotors, &imu,
 *                      3.25,           // Wheel diameter
 *                      12.0,           // Track width
 *                      600);           // Motor RPM
 *
 * // In opcontrol loop (20ms):
 * while (true) {
 *     odom.update();
 *     Pose pose = odom.getPose();
 *     pros::lcd::print(0, "X: %.2f Y: %.2f H: %.2f", pose.x, pose.y, pose.theta);
 *     pros::delay(20);
 * }
 * @endcode
 */
class Odometry {
public:
    /**
     * @brief Configuration for odometry
     *
     * Supports both motor encoders and external ADI encoders (tracking wheels).
     * External encoders are recommended for accuracy as they don't slip.
     */
    struct Config {
        // Motor groups (used if external encoders not provided)
        pros::Motor_Group* leftMotors;
        pros::Motor_Group* rightMotors;

        // External ADI encoders for tracking wheels (optional but recommended)
        // These are more accurate than motor encoders as they don't slip
        pros::ADIEncoder* leftEncoder;      // Left tracking wheel encoder
        pros::ADIEncoder* rightEncoder;     // Right tracking wheel encoder

        // Center/horizontal tracking wheel (optional, for X position tracking)
        pros::ADIEncoder* horizontalEncoder;

        pros::IMU* imu;                     // IMU for heading (required)

        float wheelDiameter;                // Wheel diameter in inches
        float trackWidth;                   // Distance between left/right wheels in inches
        float rpm;                          // Motor RPM (if using drive wheels)

        // External encoder wheel diameters (if different from drive wheels)
        float externalWheelDiameter;        // Default same as wheelDiameter

        // Horizontal wheel offset from robot center (inches, positive = forward)
        // Needed if using horizontalEncoder
        float horizontalOffset;

        Config()
            : leftMotors(nullptr), rightMotors(nullptr),
              leftEncoder(nullptr), rightEncoder(nullptr), horizontalEncoder(nullptr),
              imu(nullptr),
              wheelDiameter(3.25f), trackWidth(12.0f), rpm(600.0f),
              externalWheelDiameter(0), horizontalOffset(0) {}

        /**
         * @brief Get the effective wheel diameter
         * Returns external wheel diameter if set, otherwise drive wheel diameter
         */
        float getWheelDiameter() const {
            return externalWheelDiameter > 0 ? externalWheelDiameter : wheelDiameter;
        }
    };

    /**
     * @brief Construct odometry with configuration
     */
    explicit Odometry(const Config& config);

    /**
     * @brief Update odometry calculation
     *
     * Call this every loop iteration (20ms typical)
     */
    void update();

    /**
     * @brief Get current robot pose
     *
     * @return Pose Current x, y, theta
     */
    Pose getPose() const { return currentPose; }

    /**
     * @brief Set/reset the robot pose
     *
     * @param pose New pose to set
     *
     * Example:
     * @code
     * odom.setPose({0, 0, 90});  // Reset to origin facing +Y
     * @endcode
     */
    void setPose(const Pose& pose);

    /**
     * @brief Reset just the position (keep current heading)
     */
    void resetPosition() { currentPose.x = 0; currentPose.y = 0; }

    /**
     * @brief Get the total distance traveled since last reset
     */
    float getTotalDistance() const { return totalDistance; }

    /**
     * @brief Calculate the shortest angle difference
     *
     * @param target Target angle in degrees
     * @param current Current angle in degrees
     * @return float Shortest path (-180 to 180)
     */
    static float angleDifference(float target, float current);

    /**
     * @brief Normalize angle to 0-360 range
     */
    static float normalizeAngle(float angle);

private:
    Config config;
    Pose currentPose;
    float totalDistance;

    // Previous encoder values for delta calculation
    float prevLeftPos;
    float prevRightPos;
    float prevHorizontalPos;
    float prevHeading;

    // Conversion factors
    float ticksPerInch;         // For vertical (forward) movement
    float ticksPerInchH;        // For horizontal (strafe) movement

    float getLeftPosition();
    float getRightPosition();
    float getHorizontalPosition();
    float getHeading();

    bool firstUpdate;
};

/**
 * @brief Waypoint for path following
 */
struct Waypoint {
    float x;
    float y;
    float speed;        // Target speed at this point (0-127)
    float lookahead;      // Lookahead distance for pure pursuit (optional)

    Waypoint(float xPos, float yPos, float spd = 100, float look = 12.0f)
        : x(xPos), y(yPos), speed(spd), lookahead(look) {}
};

} // namespace Helix
