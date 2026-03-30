#pragma once

#include "Helix/PIDController.hpp"
#include "Helix/Odometry.hpp"
#include "pros/motors.hpp"
#include "pros/imu.hpp"

namespace Helix {

// Forward declarations
class Odometry;
struct Pose;

/**
 * @brief Drivetrain configuration - groups motors with physical parameters
 *
 * Example:
 * @code
 * // Create motor groups
 * pros::Motor_Group leftMotors({leftFront, leftMiddle, leftBack});
 * pros::Motor_Group rightMotors({rightFront, rightMiddle, rightBack});
 *
 * // Configure drivetrain
 * Drivetrain drivetrain(
 *     &leftMotors,      // Left motor group
 *     &rightMotors,     // Right motor group
 *     360,               // RPM (blue cartridge = 600, green = 200, red = 100)
 *     3.25               // Wheel diameter in inches
 * );
 * @endcode
 */
struct Drivetrain {
    pros::Motor_Group* leftMotors;
    pros::Motor_Group* rightMotors;
    float rpm;              // Motor RPM
    float wheelDiameter;    // In inches

    /**
     * @brief Default constructor
     */
    Drivetrain() = default;

    /**
     * @brief Construct a new Drivetrain with motor groups and physical parameters
     *
     * @param left Pointer to left motor group
     * @param right Pointer to right motor group
     * @param motorRPM Motor RPM (blue = 600, green = 200, red = 100)
     * @param wheelDiam Wheel diameter in inches
     */
    Drivetrain(pros::Motor_Group* left, pros::Motor_Group* right, float motorRPM, float wheelDiam)
        : leftMotors(left), rightMotors(right), rpm(motorRPM), wheelDiameter(wheelDiam) {}

    /**
     * @brief Calculate encoder ticks per inch of travel
     *
     * @return float Encoder ticks (in degrees) per inch
     *
     * Example:
     * @code
     * float ticks = drivetrain.ticksPerInch();  // Returns ~35.2 for 3.25" wheel
     * @endcode
     */
    float ticksPerInch() const {
        // 360 degrees per wheel rotation, wheel circumference = π * diameter
        // Using 3.14159f for π (M_PI not always available)
        return 360.0f / (wheelDiameter * 3.14159f);
    }

    /**
     * @brief Get average position of all motors in degrees
     *
     * @return float Average motor position
     */
    float getAveragePosition() const;

    /**
     * @brief Get the difference between left and right sides
     * Positive = turned right (right side traveled more), Negative = turned left
     *
     * @return float Position difference in degrees
     */
    float getPositionDifference() const;

    /**
     * @brief Reset all motor encoder positions to zero
     *
     * Example:
     * @code
     * drivetrain.tarePosition();  // Zero encoders before movement
     * @endcode
     */
    void tarePosition() const;
};

/**
 * @brief Chassis controller for autonomous movements
 *
 * Manages drivetrain with PID controllers for precise forward/backward
 * and turning movements.
 *
 * Complete example:
 * @code
 * // In main.cpp, create the chassis
 * pros::Motor leftFront(1, pros::E_MOTOR_GEAR_BLUE, true);
 * pros::Motor leftMiddle(2, pros::E_MOTOR_GEAR_BLUE, true);
 * pros::Motor leftBack(3, pros::E_MOTOR_GEAR_BLUE, true);
 * pros::Motor rightFront(4, pros::E_MOTOR_GEAR_BLUE, false);
 * pros::Motor rightMiddle(5, pros::E_MOTOR_GEAR_BLUE, false);
 * pros::Motor rightBack(6, pros::E_MOTOR_GEAR_BLUE, false);
 *
 * pros::Motor_Group leftGroup({leftFront, leftMiddle, leftBack});
 * pros::Motor_Group rightGroup({rightFront, rightMiddle, rightBack});
 * pros::IMU imu(10);
 *
 * Helix::Drivetrain drivetrain(&leftGroup, &rightGroup, 600, 3.25);
 *
 * // Configure PID controllers
 * Helix::Chassis::Config config;
 * config.drivetrain = drivetrain;
 * config.lateralPID = Helix::PIDController(0.8, 0.001, 0.1);
 * config.turnPID = Helix::PIDController(1.5, 0.002, 0.2);
 * config.maxLateralSpeed = 100;    // Limit drive speed
 * config.maxTurnSpeed = 80;        // Limit turn speed
 *
 * Helix::Chassis chassis(config);
 *
 * // In autonomous function:
 * void autonomous() {
 *     // Drive forward 24 inches
 *     chassis.drive(24);
 *
 *     // Turn 90 degrees right
 *     chassis.turn(90);
 *
 *     // Drive backward 12 inches at half speed
 *     chassis.drive(-12, 50);
 *
     // Turn to face heading 180
 *     chassis.turnTo(180);
 * }
 * @endcode
 */
class Chassis {
public:
    /**
     * @brief Configuration struct for the Chassis
     *
     * All fields have sensible defaults but should be tuned for your robot
     */
    struct Config {
        Drivetrain drivetrain;
        PIDController lateralPID = PIDConfigs::BALANCED;
        PIDController turnPID = PIDController(1.5, 0.002, 0.15);
        pros::IMU* imu = nullptr;           // Optional - for heading-based turns
        float maxLateralSpeed = 127;        // Max voltage for driving (0-127)
        float maxTurnSpeed = 127;           // Max voltage for turning (0-127)
        int defaultTimeout = 2000;          // Default movement timeout in ms
    };

    /**
     * @brief Construct a new Chassis with configuration
     *
     * @param config Chassis configuration
     *
     * Example:
     * @code
     * Chassis::Config cfg;
     * cfg.drivetrain = drivetrain;
     * cfg.lateralPID = PIDController(1.0, 0.001, 0.1);
     * cfg.imu = &imuSensor;
     *
     * Chassis chassis(cfg);
     * @endcode
     */
    explicit Chassis(const Config& config);

    /**
     * @brief Drive forward or backward a specific distance
     *
     * @param distance Distance in inches (positive = forward, negative = backward)
     * @param maxSpeed Maximum speed 0-127 (optional, uses config default if not specified)
     * @param timeout Timeout in milliseconds (optional, uses config default if not specified)
     * @return true if target reached, false if timed out
     *
     * Example:
     * @code
     * // Drive forward 24 inches
     * chassis.drive(24);
     *
     * // Drive backward 12 inches at 80% speed with 3 second timeout
     * chassis.drive(-12, 100, 3000);
     * @endcode
     */
    bool drive(float distance, float maxSpeed = -1, int timeout = -1);

    /**
     * @brief Turn relative to current heading
     *
     * @param angle Angle to turn in degrees (positive = right/clockwise)
     * @param maxSpeed Maximum speed 0-127 (optional)
     * @param timeout Timeout in milliseconds (optional)
     * @return true if target reached, false if timed out
     *
     * Example:
     * @code
     * // Turn 90 degrees right
     * chassis.turn(90);
     *
     * // Turn 45 degrees left at slower speed
     * chassis.turn(-45, 60);
     * @endcode
     */
    bool turn(float angle, float maxSpeed = -1, int timeout = -1);

    /**
     * @brief Turn to an absolute heading (requires IMU)
     *
     * @param heading Target heading in degrees (0-360)
     * @param maxSpeed Maximum speed 0-127 (optional)
     * @param timeout Timeout in milliseconds (optional)
     * @return true if target reached, false if timed out
     *
     * Example:
     * @code
     * // Turn to face North (assuming IMU calibrated)
     * chassis.turnTo(0);
     *
     * // Turn to face East
     * chassis.turnTo(90);
     * @endcode
     */
    bool turnTo(float heading, float maxSpeed = -1, int timeout = -1);

    /**
     * @brief Stop the chassis with brake mode
     *
     * @param brakeMode Brake mode to apply (default: hold)
     *
     * Example:
     * @code
     * chassis.stop();  // Stop and hold position
     * chassis.stop(pros::E_MOTOR_BRAKE_COAST);  // Stop and coast
     * @endcode
     */
    void stop(pros::motor_brake_mode_e_t brakeMode = pros::E_MOTOR_BRAKE_HOLD);

    /**
     * @brief Set the drivetrain to tank drive voltages
     *
     * @param leftVoltage Left side voltage (-127 to 127)
     * @param rightVoltage Right side voltage (-127 to 127)
     *
     * Example:
     * @code
     * // Drive straight at 50% power
     * chassis.tank(64, 64);
     *
     * // Spin in place
     * chassis.tank(64, -64);
     * @endcode
     */
    void tank(int leftVoltage, int rightVoltage);

    /**
     * @brief Arcade drive control
     *
     * @param forward Forward/backward input (-127 to 127)
     * @param turn Turn input (-127 to 127, positive = right)
     *
     * Example:
     * @code
     * // In opcontrol loop:
     * int fwd = controller.get_analog(ANALOG_LEFT_Y);
     * int turn = controller.get_analog(ANALOG_RIGHT_X);
     * chassis.arcade(fwd, turn);
     * @endcode
     */
    void arcade(int forward, int turn);

    /**
     * @brief Get the current lateral PID controller (for tuning)
     *
     * @return PIDController& Reference to lateral PID
     *
     * Example:
     * @code
     * // Tune PID during testing
     * chassis.getLateralPID().kP = 1.2;
     * @endcode
     */
    PIDController& getLateralPID() { return lateralPID_; }

    /**
     * @brief Get the current turn PID controller (for tuning)
     *
     * @return PIDController& Reference to turn PID
     */
    PIDController& getTurnPID() { return turnPID_; }

    /**
     * @brief Drive to a specific field coordinate using odometry
     *
     * @param x Target X coordinate in inches
     * @param y Target Y coordinate in inches
     * @param maxSpeed Maximum speed 0-127 (optional)
     * @param timeout Timeout in milliseconds (optional)
     * @return true if target reached, false if timed out
     *
     * Requires Odometry to be set via setOdometry()
     *
     * Example:
     * @code
     * chassis->driveToPoint(24, 24);  // Drive to field coordinate (24, 24)
     * @endcode
     */
    bool driveToPoint(float x, float y, float maxSpeed = -1, int timeout = -1);

    /**
     * @brief Turn to face a specific field coordinate
     *
     * @param x Target X coordinate in inches
     * @param y Target Y coordinate in inches
     * @param maxSpeed Maximum speed 0-127 (optional)
     * @param timeout Timeout in milliseconds (optional)
     * @return true if target reached, false if timed out
     *
     * Requires Odometry to be set via setOdometry()
     */
    bool turnToPoint(float x, float y, float maxSpeed = -1, int timeout = -1);

    /**
     * @brief Set the odometry system for position tracking
     *
     * @param odom Pointer to Odometry instance
     *
     * Example:
     * @code
     * Helix::Odometry odom(config);
     * chassis->setOdometry(&odom);
     * @endcode
     */
    void setOdometry(Odometry* odom);

    /**
     * @brief Set navigation configuration for point-to-point movements
     *
     * @param config Navigation config reference
     */
    void setNavConfig(const NavConfig& config) { navConfig_ = config; }

    /**
     * @brief Get current pose from odometry
     *
     * @return Pose Current x, y, theta (returns 0,0,0 if no odometry set)
     */
    Pose getPose() const;

    /**
     * @brief Set robot pose (requires odometry)
     *
     * @param pose New pose to set
     */
    void setPose(const Pose& pose);

private:
    Drivetrain drivetrain_;
    PIDController lateralPID_;
    PIDController turnPID_;
    pros::IMU* imu_;
    float maxLateralSpeed_;
    float maxTurnSpeed_;
    int defaultTimeout_;
    Odometry* odometry_;  // Optional odometry system
    NavConfig navConfig_;   // Navigation config for point-to-point movements

    /**
     * @brief Wait for a movement to complete with timeout
     *
     * @param pid PID controller to monitor
     * @param timeout Maximum time to wait in ms
     * @return true if settled within timeout, false if timed out
     */
    bool waitForSettle(PIDController& pid, int timeout);
};

} // namespace Helix