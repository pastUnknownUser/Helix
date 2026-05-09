#pragma once // Prevents the header from being included multiple times

namespace Helix {

    /**
     * @brief A robust, hardware-agnostic PID Controller for VEX V5.
     */
    class PID {
    private:
        // Gains
        double kP, kI, kD;

        // State limits
        double maxIntegral;
        double maxOutput;
        double tolerance;

        // Memory
        double error, prevError;
        double integral, derivative;

    public:
        /**
         * @brief Constructs a new PID controller.
         * @param p Proportional gain
         * @param i Integral gain
         * @param d Derivative gain
         */
        PID(double p, double i, double d);

        /**
         * @brief Sets the limits for the integral windup and max motor output.
         * @param max_i The maximum absolute value the integral can accumulate.
         * @param max_out The maximum output (e.g., 12.0 for V5 Volts).
         */
        void setLimits(double max_i, double max_out);

        /**
         * @brief Defines the error tolerance for the isSettled() check.
         * @param tol The acceptable distance from the target.
         */
        void setTolerance(double tol);

        /**
         * @brief Computes the necessary output based on the current state.
         * @param target The desired target value.
         * @param current The current sensor reading.
         * @return The calculated output (e.g., voltage to send to the motor).
         */
        double compute(double target, double current);

        /**
         * @brief Checks if the system has reached the target within the set tolerance.
         * @return True if settled, false otherwise.
         */
        bool isSettled();

        /**
         * @brief Resets the internal state (integral and previous error) to zero.
         * Call this before starting a new movement.
         */
        void reset();
    };

}