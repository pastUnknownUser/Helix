#include "Helix/PIDController.hpp"

namespace Helix {

PIDController::PIDController(double kP, double kI, double kD, double dt)
    : kP(kP), kI(kI), kD(kD), dt(dt),
      error(0), prevError(0), integral(0), derivative(0),
      outputMin(-127), outputMax(127),
      integralLimit(1000), tolerance(0.5), settleCount(0), settleRequired(5) {}

double PIDController::compute(double setpoint, double measurement) {
    // Calculate error
    error = setpoint - measurement;

    // Proportional term
    double pTerm = kP * error;

    // Integral term with anti-windup
    integral += error * dt;
    if (integral > integralLimit) {
        integral = integralLimit;
    } else if (integral < -integralLimit) {
        integral = -integralLimit;
    }
    double iTerm = kI * integral;

    // Derivative term (on measurement to avoid derivative kick)
    derivative = (error - prevError) / dt;
    double dTerm = kD * derivative;

    // Store error for next iteration
    prevError = error;

    // Sum terms
    double output = pTerm + iTerm + dTerm;

    // Apply output limits
    if (output > outputMax) {
        output = outputMax;
    } else if (output < outputMin) {
        output = outputMin;
    }

    // Check if settled
    if (error < 0 ? -error : error < tolerance) {
        settleCount++;
    } else {
        settleCount = 0;
    }

    return output;
}

void PIDController::reset() {
    error = 0;
    prevError = 0;
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
    settleCount = 0;
}

bool PIDController::isSettled() const {
    return settleCount >= settleRequired;
}

// Namespace pre-configured instances
namespace PIDConfigs {
    // These will be instantiated in user code
}

} // namespace Helix
