#include "Helix/MotionProfile.hpp"
#include <cmath>

namespace Helix {

MotionProfile::MotionProfile()
    : maxJerk_(40.0),
      maxAcceleration_(16.0),
      maxVelocity_(60.0),
      targetDistance_(0.0),
      totalTime_(0.0),
      t1_(0), t2_(0), t3_(0), t4_(0), t5_(0), t6_(0), t7_(0),
      d1_(0), d2_(0), d3_(0), d4_(0), d5_(0), d6_(0), d7_(0) {}

void MotionProfile::setConstraints(double maxJerk, double maxAcceleration, double maxVelocity) {
    maxJerk_ = std::abs(maxJerk);
    maxAcceleration_ = std::abs(maxAcceleration);
    maxVelocity_ = std::abs(maxVelocity);
}

bool MotionProfile::generate(double distance) {
    targetDistance_ = distance;
    double dir = sign(distance);
    double absDist = std::abs(distance);

    // Use positive values for calculation, apply direction at end
    double vMax = maxVelocity_;
    double aMax = maxAcceleration_;
    double jMax = maxJerk_;

    // Calculate phase times for S-curve acceleration
    // Phase 1: t1 = aMax / jMax (reach max acceleration)
    t1_ = aMax / jMax;
    double v1 = 0.5 * jMax * t1_ * t1_;  // Velocity at end of phase 1

    // Check if we can reach max velocity
    // Phase 3 mirrors phase 1, velocity at end of accel: v3 = v1 + aMax * t2 + v1
    // For triangular profile (no cruise), v3 = 2*v1 = aMax^2 / jMax
    double vTriangular = aMax * aMax / jMax;

    double vCruise;  // Actual cruise velocity
    double tAccel;   // Total acceleration time

    if (vTriangular >= vMax) {
        // We can reach max velocity - full profile with cruise phase
        // Phase 2: constant acceleration to reach vMax
        double vRemaining = vMax - 2 * v1;
        if (vRemaining < 0) {
            // Can't even reach acceleration limit, reduce to triangular
            vCruise = vTriangular;
            t2_ = 0;
        } else {
            t2_ = vRemaining / aMax;
            vCruise = vMax;
        }
    } else {
        // Triangular profile - never reach max velocity
        vCruise = vTriangular;
        t2_ = 0;
    }

    // Distance covered during acceleration
    // Using triangular/S-curve area approximation

    // Actually compute correctly
    // Distance during phase 1: d1 = (1/6) * jMax * t1^3
    d1_ = (1.0/6.0) * jMax * t1_ * t1_ * t1_;
    // Velocity at end of phase 1
    double velAfterT1 = 0.5 * jMax * t1_ * t1_;
    // Distance during phase 2: d2 = v1*t2 + 0.5*aMax*t2^2
    d2_ = velAfterT1 * t2_ + 0.5 * aMax * t2_ * t2_;
    // Distance during phase 3 (mirrors phase 1): d3 = velAfterT1*t1 + 0.5*aMax*t1^2 - (1/6)*jMax*t1^3
    d3_ = velAfterT1 * t1_ + 0.5 * aMax * t1_ * t1_ - (1.0/6.0) * jMax * t1_ * t1_ * t1_;

    // Simplified: distance to reach cruise velocity
    double tAccelTotal = 2 * t1_ + t2_;
    double dToCruise = 0.5 * (vCruise) * tAccelTotal;  // Triangular-ish area

    // Check if we have enough distance to reach vCruise and decelerate
    if (2 * dToCruise <= absDist) {
        // Full S-curve with cruise
        t3_ = t1_;  // Mirror phase 1
        d3_ = dToCruise - d1_ - d2_ + d1_;  // Correct this

        // Cruise distance and time
        double dCruise = absDist - 2 * dToCruise;
        t4_ = dCruise / vCruise;

        // Deceleration phases mirror acceleration
        t5_ = t3_;
        t6_ = t2_;
        t7_ = t1_;
    } else {
        // Short distance - triangular profile, no cruise
        // Need to recalculate for lower max velocity
        t2_ = 0;
        // Solve: 2 * dAccel = absDist where dAccel is area of acceleration
        // For pure S-curve without constant accel: vMax = (jMax * absDist / 2)^(1/3)
        vCruise = std::pow(jMax * absDist / 2.0, 1.0/3.0);
        t1_ = std::sqrt(vCruise / jMax);

        t3_ = t1_;
        t4_ = 0;
        t5_ = t1_;
        t6_ = 0;
        t7_ = t1_;
    }

    // Calculate total time
    totalTime_ = t1_ + t2_ + t3_ + t4_ + t5_ + t6_ + t7_;

    // Apply direction
    if (dir < 0) {
        targetDistance_ = -absDist;
    }

    return true;
}

TrajectoryPoint MotionProfile::calculate(double time) const {
    if (time <= 0) {
        return TrajectoryPoint(0, 0, 0);
    }

    if (time >= totalTime_) {
        return TrajectoryPoint(targetDistance_, 0, 0);
    }

    double t = time;
    double dir = sign(targetDistance_);

    // Phase detection
    double phaseStart = 0;
    double j = 0, a0 = 0, v0 = 0, p0 = 0;

    // Phase 1: +Jerk (increasing acceleration)
    if (t <= t1_) {
        j = maxJerk_;
        a0 = 0;
        v0 = 0;
        p0 = 0;
        phaseStart = 0;
    }
    // Phase 2: 0 Jerk (constant acceleration)
    else if (t <= t1_ + t2_) {
        j = 0;
        a0 = maxJerk_ * t1_;
        v0 = 0.5 * maxJerk_ * t1_ * t1_;
        p0 = (1.0/6.0) * maxJerk_ * t1_ * t1_ * t1_;
        phaseStart = t1_;
    }
    // Phase 3: -Jerk (decreasing acceleration)
    else if (t <= t1_ + t2_ + t3_) {
        j = -maxJerk_;
        a0 = maxJerk_ * t1_;
        v0 = 0.5 * maxJerk_ * t1_ * t1_ + maxJerk_ * t1_ * t2_;
        p0 = (1.0/6.0) * maxJerk_ * t1_ * t1_ * t1_ +
             (0.5 * maxJerk_ * t1_ * t1_) * t2_ +
             0.5 * maxJerk_ * t1_ * t2_ * t2_;
        phaseStart = t1_ + t2_;
    }
    // Phase 4: Cruise
    else if (t <= t1_ + t2_ + t3_ + t4_) {
        double dt = t - (t1_ + t2_ + t3_);
        double vCruise = 0.5 * maxJerk_ * t1_ * t1_ + maxJerk_ * t1_ * t2_ + 0.5 * maxJerk_ * t1_ * t1_;
        double dAccel = 0;  // Calculate once and cache

        // Distance covered in accel phases
        dAccel = (1.0/6.0) * maxJerk_ * t1_ * t1_ * t1_ +
                 (0.5 * maxJerk_ * t1_ * t1_) * t2_ + 0.5 * maxJerk_ * t1_ * t2_ * t2_ +
                 (0.5 * maxJerk_ * t1_ * t1_ + maxJerk_ * t1_ * t2_) * t3_ +
                 0.5 * (-maxJerk_) * t3_ * t3_ * t3_;

        double pos = dAccel + vCruise * dt;
        return TrajectoryPoint(dir * pos, dir * vCruise, 0);
    }
    // Phase 5: -Jerk (start deceleration)
    else if (t <= t1_ + t2_ + t3_ + t4_ + t5_) {
        double tDecelStart = t1_ + t2_ + t3_ + t4_;
        double vCruise = maxJerk_ * t1_ * t1_ + maxJerk_ * t1_ * t2_;
        double dCruise = vCruise * t4_;
        double dAccel = vCruise * (t1_ + t2_ + t3_) * 0.5;

        j = -maxJerk_;
        a0 = 0;
        v0 = vCruise;
        p0 = dAccel + dCruise;
        phaseStart = tDecelStart;
    }
    // Phase 6: 0 Jerk (constant deceleration)
    else if (t <= t1_ + t2_ + t3_ + t4_ + t5_ + t6_) {
        double tDecelStart = t1_ + t2_ + t3_ + t4_;
        double vCruise = maxJerk_ * t1_ * t1_ + maxJerk_ * t1_ * t2_;

        j = 0;
        a0 = -maxJerk_ * t5_;
        v0 = vCruise - 0.5 * maxJerk_ * t5_ * t5_;
        p0 = vCruise * (t1_ + t2_ + t3_ + t4_) * 0.5 +
             vCruise * t5_ - (1.0/6.0) * maxJerk_ * t5_ * t5_ * t5_;
        phaseStart = tDecelStart + t5_;
    }
    // Phase 7: +Jerk (decreasing deceleration)
    else {
        double tDecelStart = t1_ + t2_ + t3_ + t4_;
        double vCruise = maxJerk_ * t1_ * t1_ + maxJerk_ * t1_ * t2_;

        j = maxJerk_;
        a0 = -maxJerk_ * t5_;
        v0 = vCruise - 0.5 * maxJerk_ * t5_ * t5_ - maxJerk_ * t5_ * t6_;
        p0 = vCruise * (t1_ + t2_ + t3_ + t4_) * 0.5 +
             vCruise * t5_ - (1.0/6.0) * maxJerk_ * t5_ * t5_ * t5_ +
             (vCruise - 0.5 * maxJerk_ * t5_ * t5_) * t6_ -
             0.5 * maxJerk_ * t5_ * t6_ * t6_;
        phaseStart = tDecelStart + t5_ + t6_;
    }

    // Calculate within phase
    double dt = t - phaseStart;
    double accel = a0 + j * dt;
    double velocity = v0 + a0 * dt + 0.5 * j * dt * dt;
    double position = p0 + v0 * dt + 0.5 * a0 * dt * dt + (1.0/6.0) * j * dt * dt * dt;

    return TrajectoryPoint(dir * position, dir * velocity, dir * accel);
}

void MotionProfile::reset() {
    targetDistance_ = 0;
    totalTime_ = 0;
    t1_ = t2_ = t3_ = t4_ = t5_ = t6_ = t7_ = 0;
    d1_ = d2_ = d3_ = d4_ = d5_ = d6_ = d7_ = 0;
}

} // namespace Helix
