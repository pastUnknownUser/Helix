#pragma once

#include <cmath>

namespace Helix {

/**
 * @brief Trajectory point containing position, velocity, and acceleration
 *
 * Used by motion profile followers to track a desired motion.
 */
struct TrajectoryPoint {
    double position;      // Target position
    double velocity;      // Target velocity
    double acceleration;  // Target acceleration

    TrajectoryPoint(double p = 0, double v = 0, double a = 0)
        : position(p), velocity(v), acceleration(a) {}
};

/**
 * @brief S-Curve motion profile generator
 *
 * Generates smooth trajectories with limited jerk, acceleration, and velocity.
 * S-curve profiles reduce vibrations and wheel slip compared to trapezoidal profiles.
 *
 * The profile consists of 7 phases:
 * 1. +Jerk (increasing acceleration)
 * 2. 0 Jerk (constant acceleration)
 * 3. -Jerk (decreasing acceleration)
 * 4. Constant velocity (cruise)
 * 5. -Jerk (increasing deceleration)
 * 6. 0 Jerk (constant deceleration)
 * 7. +Jerk (decreasing deceleration)
 *
 * Example:
 * @code
 * MotionProfile profile;
 * profile.setConstraints(10.0, 30.0, 60.0);  // jerk, accel, vel
 * profile.generate(24.0);  // 24 inches
 *
 * while (!profile.isFinished()) {
 *     auto point = profile.calculate(currentTime);
 *     // Follow point...
 * }
 * @endcode
 */
class MotionProfile {
public:
    /**
     * @brief Construct a new motion profile
     */
    MotionProfile();

    /**
     * @brief Set motion constraints
     *
     * @param maxJerk Maximum jerk (units/sec^3)
     * @param maxAcceleration Maximum acceleration (units/sec^2)
     * @param maxVelocity Maximum velocity (units/sec)
     *
     * Tuning guide:
     * - Start with conservative values to avoid wheel slip
     * - jerk ≈ acceleration * 5-10 (higher = smoother transitions)
     * - acceleration determines how fast you speed up
     * - velocity limits top speed
     */
    void setConstraints(double maxJerk, double maxAcceleration, double maxVelocity);

    /**
     * @brief Generate profile for a given distance
     *
     * @param distance Target distance (can be negative for reverse)
     * @return true if profile generated successfully
     */
    bool generate(double distance);

    /**
     * @brief Calculate trajectory point at given time
     *
     * @param time Current time since profile start (seconds)
     * @return TrajectoryPoint Target position, velocity, and acceleration
     */
    TrajectoryPoint calculate(double time) const;

    /**
     * @brief Get total duration of the profile
     *
     * @return double Total time in seconds
     */
    double getDuration() const { return totalTime_; }

    /**
     * @brief Check if profile is finished
     *
     * @param currentTime Current elapsed time
     * @return true if profile complete
     */
    bool isFinished(double currentTime) const { return currentTime >= totalTime_; }

    /**
     * @brief Reset the profile for reuse
     */
    void reset();

    /**
     * @brief Get target distance
     */
    double getTargetDistance() const { return targetDistance_; }

private:
    // Motion constraints
    double maxJerk_;
    double maxAcceleration_;
    double maxVelocity_;

    // Profile parameters (calculated in generate)
    double targetDistance_;
    double totalTime_;

    // Phase durations
    double t1_, t2_, t3_;  // Acceleration phases
    double t4_;            // Cruise phase
    double t5_, t6_, t7_;  // Deceleration phases

    // Phase distances
    double d1_, d2_, d3_, d4_, d5_, d6_, d7_;

    // Helper calculations
    double calculatePhasePosition(double t, double tStart,
                                   double v0, double a0, double j) const;
    double calculatePhaseVelocity(double t, double tStart,
                                   double a0, double j) const;
    double calculatePhaseAcceleration(double t, double tStart, double j) const;

    // Sign helper
    static double sign(double x) { return (x > 0) ? 1.0 : ((x < 0) ? -1.0 : 0.0); }
};

/**
 * @brief Pre-tuned motion profile configurations
 *
 * These are starting points - tune for your specific robot!
 */
namespace MotionProfiles {
    /**
     * Conservative profile - prioritize accuracy over speed
     * Good for: Precise movements, heavy robots, slippery surfaces
     */
    inline MotionProfile CONSERVATIVE() {
        MotionProfile mp;
        mp.setConstraints(20.0, 8.0, 40.0);   // jerk, accel, vel
        return mp;
    }

    /**
     * Balanced profile - good starting point for most robots
     * Good for: General autonomous routines
     */
    inline MotionProfile BALANCED() {
        MotionProfile mp;
        mp.setConstraints(40.0, 16.0, 60.0);
        return mp;
    }

    /**
     * Aggressive profile - prioritize speed
     * Good for: Light robots, high-traction surfaces
     * May cause overshoot or wheel slip
     */
    inline MotionProfile AGGRESSIVE() {
        MotionProfile mp;
        mp.setConstraints(80.0, 32.0, 80.0);
        return mp;
    }
}

} // namespace Helix
