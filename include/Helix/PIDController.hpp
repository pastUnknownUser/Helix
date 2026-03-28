#pragma once

namespace Helix {

/**
 * @brief Configurable PID controller for precise motor control
 *
 * Example usage:
 * @code
 * // Create a PID for driving straight
 * PIDController drivePID(0.8, 0.001, 0.1);
 * drivePID.setOutputLimits(-127, 127);  // Voltage limits
 *
 * while (!drivePID.isSettled()) {
 *     double output = drivePID.compute(targetDistance, currentDistance);
 *     motors.move_voltage(output * 1000);  // Scale to mV
 *     pros::delay(20);
 * }
 * @endcode
 */
class PIDController {
public:
    /**
     * @brief Construct a new PID controller
     *
     * @param kP Proportional gain - responds to current error
     * @param kI Integral gain - eliminates steady-state error over time
     * @param kD Derivative gain - dampens oscillations
     * @param dt Time step in seconds (default: 0.02 = 20ms, typical PROS loop time)
     *
     * Tuning tips:
     * - Start with kP only, increase until robot oscillates
     * - Add kD to reduce oscillation (usually kP/10)
     * - Add kI if there's steady-state error (start very small, like kP/100)
     */
    PIDController(double kP, double kI, double kD, double dt = 0.02);

    /**
     * @brief Compute PID output for this timestep
     *
     * @param setpoint Target value (e.g., desired distance in inches)
     * @param measurement Current sensor reading (e.g., current distance)
     * @return double Control output to send to motors
     *
     * Example:
     * @code
     * double output = pid.compute(24.0, currentPosition);  // Drive to 24 inches
     * leftMotors.move_voltage(output * 100);
     * rightMotors.move_voltage(output * 100);
     * @endcode
     */
    double compute(double setpoint, double measurement);

    /**
     * @brief Reset the controller (call when starting a new movement)
     *
     * Example:
     * @code
     * pid.reset();  // Clear accumulated integral and previous error
     * @endcode
     */
    void reset();

    /**
     * @brief Set output limits to prevent excessive motor values
     *
     * @param min Minimum output (e.g., -127 for voltage mode)
     * @param max Maximum output (e.g., 127 for voltage mode)
     *
     * Example:
     * @code
     * pid.setOutputLimits(-80, 80);  // Limit to 80% max power
     * @endcode
     */
    void setOutputLimits(double min, double max);

    /**
     * @brief Set integral limit to prevent windup
     *
     * @param limit Absolute value limit for integral term
     *
     * Example:
     * @code
     * pid.setIntegralLimit(1000);  // Prevent integral from growing too large
     * @endcode
     */
    void setIntegralLimit(double limit);

    /**
     * @brief Set tolerance for considering the target "reached"
     *
     * @param tolerance Error magnitude considered acceptable
     * @param consecutiveSamples Number of consecutive readings within tolerance (prevents false positives)
     *
     * Example:
     * @code
     * pid.setTolerance(0.5, 3);  // Within 0.5 inches for 3 samples = settled
     * @endcode
     */
    void setTolerance(double tolerance, int consecutiveSamples = 5);

    /**
     * @brief Check if the controller has settled within tolerance
     *
     * @return true if error has been within tolerance for required samples
     *
     * Example:
     * @code
     * while (!pid.isSettled()) {
     *     // Continue PID loop
     * }
     * @endcode
     */
    bool isSettled() const;

    /**
     * @brief Get the current error
     * Useful for debugging or custom settle logic
     */
    double getError() const { return error; }

    /**
     * @brief Get the computed integral term
     * Useful for debugging integral windup
     */
    double getIntegral() const { return integral; }

    /**
     * @brief Get the computed derivative term
     * Useful for debugging oscillations
     */
    double getDerivative() const { return derivative; }

    // Tunable gains - modify these directly or use the setters
    double kP;
    double kI;
    double kD;

private:
    double dt;                  // Time step
    double error;               // Current error
    double prevError;           // Previous error for derivative
    double integral;            // Accumulated error
    double derivative;          // Rate of change
    double outputMin;           // Output limits
    double outputMax;
    double integralLimit;       // Anti-windup limit
    double tolerance;           // Acceptable error
    int settleCount;            // Consecutive samples within tolerance
    int settleRequired;         // Required samples to be settled
};

/**
 * @brief Pre-tuned PID configurations for common use cases
 *
 * These are starting points - always tune for your specific robot!
 * Heavier robots need lower gains, lighter robots can use higher gains.
 */
namespace PIDConfigs {
    /**
     * Conservative settings for precise movements
     * Use when accuracy is more important than speed
     *
     * Example:
     * @code
     * PIDController pid = PIDConfigs::PRECISE;
     * @endcode
     */
    inline PIDController PRECISE(0.5, 0.0005, 0.05);

    /**
     * Aggressive settings for fast movements
     * Use when speed is more important than precision
     * May cause overshoot on heavy robots
     *
     * Example:
     * @code
     * PIDController pid = PIDConfigs::FAST;
     * @endcode
     */
    inline PIDController FAST(1.2, 0.001, 0.15);

    /**
     * Balanced settings - good starting point for most robots
     *
     * Example:
     * @code
     * PIDController pid = PIDConfigs::BALANCED;
     * @endcode
     */
    inline PIDController BALANCED(0.8, 0.001, 0.1);
}

} // namespace Helix