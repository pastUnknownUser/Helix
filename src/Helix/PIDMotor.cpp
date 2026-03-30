#include "Helix/PIDMotor.hpp"
#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include "pros/rtos.hpp"
#include <cmath>

namespace Helix {

// Constructor with single motor (internal encoder)
PIDMotor::PIDMotor(pros::Motor* motor,
                   const PIDController& pid,
                   float gearRatio)
    : motor_(motor),
      motors_(nullptr),
      isGroup_(false),
      encoder_(nullptr),
      pid_(pid),
      gearRatio_(gearRatio),
      outputMin_(-127),
      outputMax_(127),
      target_(0),
      currentPos_(0),
      enabled_(true),
      settled_(true) {
    // Motor encoder: 1 degree = 1 tick (PROS reports in degrees)
    ticksPerDegree_ = 1.0f / gearRatio_;

    // Start background task
    task_ = std::make_unique<pros::Task>(taskLoop, this, "PIDMotor");
}

// Constructor with motor group (internal encoders)
PIDMotor::PIDMotor(pros::Motor_Group* motors,
                   const PIDController& pid,
                   float gearRatio)
    : motor_(nullptr),
      motors_(motors),
      isGroup_(true),
      encoder_(nullptr),
      pid_(pid),
      gearRatio_(gearRatio),
      outputMin_(-127),
      outputMax_(127),
      target_(0),
      currentPos_(0),
      enabled_(true),
      settled_(true) {
    // Motor encoder: 1 degree = 1 tick
    ticksPerDegree_ = 1.0f / gearRatio_;

    // Start background task
    task_ = std::make_unique<pros::Task>(taskLoop, this, "PIDMotorGroup");
}

// Constructor with single motor and external encoder
PIDMotor::PIDMotor(pros::Motor* motor,
                   const PIDController& pid,
                   pros::ADIEncoder* encoder,
                   float ticksPerRevolution,
                   float gearRatio)
    : motor_(motor),
      motors_(nullptr),
      isGroup_(false),
      encoder_(encoder),
      pid_(pid),
      gearRatio_(gearRatio),
      outputMin_(-127),
      outputMax_(127),
      target_(0),
      currentPos_(0),
      enabled_(true),
      settled_(true) {
    // External encoder: calculate ticks per degree
    ticksPerDegree_ = (ticksPerRevolution / 360.0f) / gearRatio_;

    task_ = std::make_unique<pros::Task>(taskLoop, this, "PIDMotorExt");
}

// Constructor with motor group and external encoder
PIDMotor::PIDMotor(pros::Motor_Group* motors,
                   const PIDController& pid,
                   pros::ADIEncoder* encoder,
                   float ticksPerRevolution,
                   float gearRatio)
    : motor_(nullptr),
      motors_(motors),
      isGroup_(true),
      encoder_(encoder),
      pid_(pid),
      gearRatio_(gearRatio),
      outputMin_(-127),
      outputMax_(127),
      target_(0),
      currentPos_(0),
      enabled_(true),
      settled_(true) {
    // External encoder: calculate ticks per degree
    ticksPerDegree_ = (ticksPerRevolution / 360.0f) / gearRatio_;

    task_ = std::make_unique<pros::Task>(taskLoop, this, "PIDMotorGrpExt");
}

// Destructor
PIDMotor::~PIDMotor() {
    enabled_ = false;
    if (task_) {
        task_->remove();
    }
    stop();
}

// Set target position (the main API)
void PIDMotor::setPID(float target) {
    target_ = target;
    pid_.reset();
    settled_ = false;
}

// Set relative position
void PIDMotor::setPIDRelative(float relativeAngle) {
    float current = getPosition();
    setPID(current + relativeAngle);
}

// Block until settled
bool PIDMotor::waitUntilSettled(int timeout) {
    int elapsed = 0;
    const int dt = 20;

    while (!isSettled() && elapsed < timeout) {
        pros::delay(dt);
        elapsed += dt;
    }

    return isSettled();
}

// Check if settled
bool PIDMotor::isSettled() const {
    return settled_.load();
}

// Get current position (output shaft, after gear ratio)
float PIDMotor::getPosition() const {
    float raw = readPosition();
    // Convert to output degrees (accounting for gear ratio)
    return raw / gearRatio_;
}

// Tare encoder
void PIDMotor::tarePosition() {
    if (encoder_) {
        encoder_->reset();
    } else if (isGroup_) {
        if (motors_) motors_->tare_position();
    } else {
        if (motor_) motor_->tare_position();
    }
}

// Stop motor
void PIDMotor::stop(pros::motor_brake_mode_e_t brakeMode) {
    if (isGroup_) {
        if (motors_) {
            motors_->move(0);
            motors_->set_brake_modes(brakeMode);
        }
    } else {
        if (motor_) {
            motor_->move(0);
            motor_->set_brake_mode(brakeMode);
        }
    }
}

// Direct voltage control (bypass PID)
void PIDMotor::move(int voltage) {
    settled_ = true;  // Cancel PID
    writeOutput(voltage);
}

// Set output limits
void PIDMotor::setOutputLimits(int min, int max) {
    outputMin_ = min;
    outputMax_ = max;
    pid_.setOutputLimits(min, max);
}

// Set tolerance
void PIDMotor::setTolerance(float tolerance, int samples) {
    pid_.setTolerance(tolerance, samples);
}

// Background task loop
void PIDMotor::taskLoop(void* params) {
    PIDMotor* instance = static_cast<PIDMotor*>(params);

    while (instance->enabled_.load()) {
        instance->update();
        pros::delay(20);  // 50Hz
    }
}

// Main update loop
void PIDMotor::update() {
    // Read current position
    currentPos_ = readPosition();

    // If settled, don't compute
    if (settled_.load()) {
        return;
    }

    // Compute PID
    float target = target_.load();
    float output = pid_.compute(target, currentPos_.load());

    // Apply to motor
    writeOutput(static_cast<int>(output));

    // Check if settled
    if (pid_.isSettled()) {
        settled_ = true;
    }
}

// Read position from encoder (motor or external)
float PIDMotor::readPosition() const {
    if (encoder_) {
        // External encoder returns ticks
        float ticks = encoder_->get_value();
        return ticksToDegrees(ticks);
    } else if (isGroup_) {
        // Average all motor positions
        if (motors_) {
            auto positions = motors_->get_positions();
            float sum = 0;
            for (double pos : positions) {
                sum += static_cast<float>(pos);
            }
            return sum / positions.size();
        }
    } else {
        // Single motor
        if (motor_) {
            return motor_->get_position();
        }
    }
    return 0;
}

// Write output to motor(s)
void PIDMotor::writeOutput(int output) {
    // Clamp to limits
    if (output > outputMax_) output = outputMax_;
    if (output < outputMin_) output = outputMin_;

    if (isGroup_) {
        if (motors_) motors_->move(output);
    } else {
        if (motor_) motor_->move(output);
    }
}

// Convert encoder ticks to degrees (output shaft)
float PIDMotor::ticksToDegrees(float ticks) const {
    return ticks / ticksPerDegree_ / gearRatio_;
}

// Convert degrees to encoder ticks
float PIDMotor::degreesToTicks(float degrees) const {
    return degrees * ticksPerDegree_ * gearRatio_;
}

} // namespace Helix
