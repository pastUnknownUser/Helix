#pragma once

#include "Helix/PIDController.hpp"
#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include "pros/rtos.hpp"
#include <memory>
#include <atomic>

namespace Helix {

/**
 * @brief PID-controlled motor wrapper for any motor or motor group
 *
 * Wraps a single motor or motor group with PID control and optional
 * external encoder feedback. Provides simple position/angle control
 * with automatic PID computation in a background task.
 *
 * Example - Single motor with internal encoder:
 * @code
 * pros::Motor armMotor(5, pros::E_MOTOR_GEAR_GREEN, false);
 *
 * Helix::PIDMotor arm(&armMotor,
 *     Helix::PIDController(0.5, 0.001, 0.1)  // PID gains
 * );
 *
 * // In autonomous - moves to 90 degrees and waits
 * arm.setPID(90);           // Start PID to 90 degrees
 * arm.waitUntilSettled();   // Block until done
 * @endcode
 *
 * Example - With external encoder (more accurate):
 * @code
 * pros::Motor intakeMotor(7, pros::E_MOTOR_GEAR_BLUE, false);
 * pros::ADIEncoder encoder('A', 'B', false);  // External encoder
 *
 * Helix::PIDMotor intake(
 *     &intakeMotor,
 *     Helix::PIDController(0.8, 0.002, 0.15),
 *     &encoder,           // Use external encoder for feedback
 *     360.0               // Ticks per revolution (360 for ADI encoder)
 * );
 *
 * intake.setPID(720);      // Spin 2 rotations
 * intake.waitUntilSettled();
 * @endcode
 *
 * Example - Non-blocking usage:
 * @code
 * arm.setPID(45);           // Start moving
 * // Do other things...
 * if (arm.isSettled()) {    // Check if done
 *     arm.stop();
 * }
 * @endcode
 */
class PIDMotor {
public:
    /**
     * @brief Construct PIDMotor with motor (uses motor's internal encoder)
     *
     * @param motor Pointer to motor (single motor)
     * @param pid PIDController with tuned gains
     * @param gearRatio Gear ratio if motor is behind a gearbox (default 1.0)
     *                  e.g., 5:1 ratio = 5.0, 1:7 ratio = 7.0
     */
    PIDMotor(pros::Motor* motor,
             const PIDController& pid,
             float gearRatio = 1.0f);

    /**
     * @brief Construct PIDMotor with motor group (uses average of motor encoders)
     *
     * @param motors Pointer to motor group
     * @param pid PIDController with tuned gains
     * @param gearRatio Gear ratio if motors are behind a gearbox
     */
    PIDMotor(pros::Motor_Group* motors,
             const PIDController& pid,
             float gearRatio = 1.0f);

    /**
     * @brief Construct PIDMotor with external encoder
     *
     * Uses external encoder for position feedback instead of motor encoders.
     * This is more accurate for mechanisms with slip (like intakes).
     *
     * @param motor Pointer to motor or motor group
     * @param pid PIDController with tuned gains
     * @param encoder Pointer to external ADI encoder
     * @param ticksPerRevolution Encoder ticks per output revolution
     *                           (360 for standard V5 ADI encoder)
     * @param gearRatio Additional gear ratio between encoder and output
     */
    PIDMotor(pros::Motor* motor,
             const PIDController& pid,
             pros::ADIEncoder* encoder,
             float ticksPerRevolution,
             float gearRatio = 1.0f);

    /**
     * @brief Construct PIDMotor with motor group and external encoder
     */
    PIDMotor(pros::Motor_Group* motors,
             const PIDController& pid,
             pros::ADIEncoder* encoder,
             float ticksPerRevolution,
             float gearRatio = 1.0f);

    /**
     * @brief Destructor - stops background task
     */
    ~PIDMotor();

    /**
     * @brief Move to target position (non-blocking)
     *
     * Starts PID control to reach the target position. Call
     * waitUntilSettled() to block until complete, or check isSettled()
     * periodically.
     *
     * @param target Target position in degrees (motor shaft or output,
     *               depending on gearRatio)
     *
     * Example:
     * @code
     * arm.setPID(90);    // Move arm to 90 degrees
     * arm.waitUntilSettled();  // Wait for completion
     * @endcode
     */
    void setPID(float target);

    /**
     * @brief Move relative to current position
     *
     * @param relativeAngle Angle to move relative to current position
     *
     * Example:
     * @code
     * arm.setPIDRelative(45);   // Move 45 degrees from current position
     * @endcode
     */
    void setPIDRelative(float relativeAngle);

    /**
     * @brief Block until PID settles or timeout
     *
     * @param timeout Maximum time to wait in ms (default 5000)
     * @return true if settled, false if timed out
     */
    bool waitUntilSettled(int timeout = 5000);

    /**
     * @brief Check if PID has settled
     *
     * @return true if within tolerance for required samples
     */
    bool isSettled() const;

    /**
     * @brief Get current position
     *
     * @return float Current position in degrees
     */
    float getPosition() const;

    /**
     * @brief Get target position
     *
     * @return float Target position in degrees
     */
    float getTarget() const { return target_.load(); }

    /**
     * @brief Reset encoder position to zero
     */
    void tarePosition();

    /**
     * @brief Stop the motor
     *
     * @param brakeMode Brake mode to apply (default: hold)
     */
    void stop(pros::motor_brake_mode_e_t brakeMode = pros::E_MOTOR_BRAKE_HOLD);

    /**
     * @brief Direct voltage control (bypasses PID)
     *
     * @param voltage Voltage -127 to 127
     */
    void move(int voltage);

    /**
     * @brief Get the PID controller (for tuning)
     *
     * @return PIDController& Reference to PID controller
     */
    PIDController& getPID() { return pid_; }

    /**
     * @brief Set output limits
     *
     * @param min Minimum output (-127 to 0)
     * @param max Maximum output (0 to 127)
     */
    void setOutputLimits(int min, int max);

    /**
     * @brief Set tolerance for settle detection
     *
     * @param tolerance Position error considered acceptable (degrees)
     * @param samples Consecutive samples within tolerance (default 5)
     */
    void setTolerance(float tolerance, int samples = 5);

private:
    // Motor references (only one of these will be set)
    pros::Motor* motor_;
    pros::Motor_Group* motors_;
    bool isGroup_;

    // Optional external encoder
    pros::ADIEncoder* encoder_;

    // Configuration
    PIDController pid_;
    float ticksPerDegree_;      // Conversion from degrees to encoder ticks
    float gearRatio_;
    int outputMin_;
    int outputMax_;

    // State
    std::atomic<float> target_;
    std::atomic<float> currentPos_;
    std::atomic<bool> enabled_;
    std::atomic<bool> settled_;

    // Background task
    std::unique_ptr<pros::Task> task_;
    static void taskLoop(void* params);
    void update();

    // Helpers
    float readPosition() const;
    void writeOutput(int output);
    float ticksToDegrees(float ticks) const;
    float degreesToTicks(float degrees) const;
};

/**
 * @brief Configuration presets for common mechanisms
 */
namespace PIDMotorConfigs {
    /**
     * @brief Precise arm control (slower, minimal overshoot)
     */
    inline PIDController ARM(0.5f, 0.0005f, 0.1f);

    /**
     * @brief Fast intake/roller (aggressive, may overshoot)
     */
    inline PIDController INTAKE(1.2f, 0.002f, 0.15f);

    /**
     * @brief Balanced lift (good starting point)
     */
    inline PIDController LIFT(0.8f, 0.001f, 0.1f);

    /**
     * @brief Slow but precise turret
     */
    inline PIDController TURRET(0.3f, 0.0001f, 0.05f);
}

} // namespace Helix
