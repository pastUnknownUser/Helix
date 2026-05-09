#include "helixApi.h"

#include "PID.h"
#include <cmath>

namespace Helix {

    PID::PID(double p, double i, double d) 
        : kP(p), kI(i), kD(d), maxIntegral(0), maxOutput(12.0), tolerance(1.0),
          error(0), prevError(0), integral(0), derivative(0) {}

    void PID::setLimits(double max_i, double max_out) {
        maxIntegral = max_i;
        maxOutput = max_out;
    }

    void PID::setTolerance(double tol) {
        tolerance = tol;
    }

    double PID::compute(double target, double current) {
        error = target - current;

        double p_out = error * kP;

        integral += error;
        if (maxIntegral > 0) {
            if (integral > maxIntegral) integral = maxIntegral;
            else if (integral < -maxIntegral) integral = -maxIntegral;
        }
        
        if ((error > 0 && prevError < 0) || (error < 0 && prevError > 0) || error == 0) {
            integral = 0;
        }
        double i_out = integral * kI;

        derivative = error - prevError;
        double d_out = derivative * kD;

        prevError = error;

        double output = p_out + i_out + d_out;

        if (maxOutput > 0) {
            if (output > maxOutput) output = maxOutput;
            else if (output < -maxOutput) output = -maxOutput;
        }

        return output;
    }

    bool PID::isSettled() {
        return (std::abs(error) < tolerance) && (std::abs(derivative) < tolerance);
    }

    void PID::reset() {
        error = 0;
        prevError = 0;
        integral = 0;
        derivative = 0;
    }

}