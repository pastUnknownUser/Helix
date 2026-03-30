#include "Helix/PIDController.hpp"
#include <cmath>

namespace Helix {

PIDController::PIDController(double kP, double kI, double kD, double dt)
    : kP(kP), kI(kI), kD(kD), dt(dt),
      error(0), prevError(0), prevMeasurement(0), integral(0), derivative(0),
      outputMin(-127), outputMax(127),
      integralLimit(1000), tolerance(0.5), settleCount(0), settleRequired(5) {}

double PIDController::compute(double setpoint, double measurement) {
    // Calculate error
    error = setpoint - measurement;

    // Proportional term
    double pTerm = kP * error;

    // Integral term with anti-windup (only integrate if not saturated)
    double outputWithoutI = kP * error + kD * ((error - prevError) / dt);
    bool saturated = outputWithoutI > outputMax || outputWithoutI < outputMin;

    if (!saturated || std::abs(error) < tolerance) {
        integral += error * dt;
        if (integral > integralLimit) {
            integral = integralLimit;
        } else if (integral < -integralLimit) {
            integral = -integralLimit;
        }
    }
    double iTerm = kI * integral;

    // Derivative term on MEASUREMENT (not error) to avoid derivative kick
    // This computes: -d(measurement)/dt instead of d(error)/dt
    // Since error = setpoint - measurement, d(error)/dt = -d(measurement)/dt
    derivative = (measurement - prevMeasurement) / dt;
    double dTerm = -kD * derivative;

    // Store for next iteration
    prevError = error;
    prevMeasurement = measurement;

    // Sum terms
    double output = pTerm + iTerm + dTerm;

    // Apply output limits
    if (output > outputMax) {
        output = outputMax;
    } else if (output < outputMin) {
        output = outputMin;
    }

    // Check if settled
    if (std::abs(error) < tolerance) {
        settleCount++;
    } else {
        settleCount = 0;
    }

    return output;
}

void PIDController::reset() {
    error = 0;
    prevError = 0;
    prevMeasurement = 0;
    integral = 0;
    derivative = 0;
    settleCount = 0;
}

void PIDController::setOutputLimits(double min, double max) {
    outputMin = min;
    outputMax = max;
}

void PIDController::setIntegralLimit(double limit) {
    integralLimit = limit < 0 ? -limit : limit;
}

void PIDController::setTolerance(double tolerance_, int consecutiveSamples) {
    tolerance = tolerance_ < 0 ? -tolerance_ : tolerance_;
    settleRequired = consecutiveSamples;
}

bool PIDController::isSettled() const {
    return settleCount >= settleRequired;
}

// Namespace pre-configured instances
namespace PIDConfigs {
    // These will be instantiated in user code
}

} // namespace Helix
