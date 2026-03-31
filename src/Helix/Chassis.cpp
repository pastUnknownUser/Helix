#include "Helix/Chassis.hpp"
#include "Helix/Odometry.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <vector>

// Define M_PI if not available (some platforms require _USE_MATH_DEFINES)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace Helix {

namespace {
    // Helper to average a vector of positions
    float averagePosition(const std::vector<double>& positions) {
        if (positions.empty()) return 0.0f;
        double sum = 0;
        for (double pos : positions) {
            sum += pos;
        }
        return static_cast<float>(sum / positions.size());
    }

    // Get current time in milliseconds
    float getCurrentTime() {
        return static_cast<float>(pros::millis());
    }
}

// Drivetrain implementation
float Drivetrain::getAveragePosition() const {
    if (!leftMotors || !rightMotors) return 0.0f;

    float leftAvg = averagePosition(leftMotors->get_positions());
    float rightAvg = averagePosition(rightMotors->get_positions());

    return (leftAvg + rightAvg) / 2.0f;
}

float Drivetrain::getPositionDifference() const {
    if (!leftMotors || !rightMotors) return 0.0f;

    float leftAvg = averagePosition(leftMotors->get_positions());
    float rightAvg = averagePosition(rightMotors->get_positions());

    // Positive = turned right (left traveled more than right)
    return leftAvg - rightAvg;
}

void Drivetrain::tarePosition() const {
    if (leftMotors) leftMotors->tare_position();
    if (rightMotors) rightMotors->tare_position();
}

// Chassis implementation
Chassis::Chassis(const Config& config)
    : drivetrain_(config.drivetrain),
      lateralPID_(config.lateralPID.kP, config.lateralPID.kI, config.lateralPID.kD),
      turnPID_(config.turnPID.kP, config.turnPID.kI, config.turnPID.kD),
      imu_(config.imu),
      maxLateralSpeed_(config.maxLateralSpeed),
      maxTurnSpeed_(config.maxTurnSpeed),
      defaultTimeout_(config.defaultTimeout),
      odometry_(nullptr),
      // Feedforward
      lateralkV_(config.lateralkV),
      lateralkA_(config.lateralkA),
      lateralkS_(config.lateralkS),
      turnkV_(config.turnkV),
      turnkA_(config.turnkA),
      turnkS_(config.turnkS),
      // Motion profiling
      useMotionProfile_(config.useMotionProfile),
      // Slew rate
      slewRate_(config.slewRate),
      lastLeftOutput_(0),
      lastRightOutput_(0),
      // Async state
      currentMotion_(MotionType::NONE),
      motionSettled_(true),
      motionTarget_(0),
      motionStartTime_(0),
      motionTimeout_(0),
      motionMaxSpeed_(0) {

    // Set up motion profile constraints
    if (useMotionProfile_) {
        lateralProfile_.setConstraints(config.maxJerk, config.maxAcceleration, config.maxVelocity);
        turnProfile_.setConstraints(config.maxJerk * 2, config.maxAcceleration * 2, config.maxVelocity);
    }
}

double Chassis::applySlewRate(double target, double& lastOutput) {
    if (slewRate_ <= 0.0) {
        lastOutput = target;
        return target;
    }

    double maxChange = slewRate_ * 0.02; // 20ms loop time
    double change = target - lastOutput;

    if (change > maxChange) {
        change = maxChange;
    } else if (change < -maxChange) {
        change = -maxChange;
    }

    lastOutput += change;
    return lastOutput;
}

double Chassis::calculateFeedforward(double velocity, double acceleration,
                                     double kV, double kA, double kS) {
    double ff = kV * velocity + kA * acceleration;
    if (std::abs(velocity) > 0.01) {
        ff += kS * (velocity > 0 ? 1.0 : -1.0);
    }
    return ff;
}

bool Chassis::executeDrive(float distance, float maxSpeed, int timeout) {
    // Reset PID and encoders
    lateralPID_.reset();
    drivetrain_.tarePosition();
    lastLeftOutput_ = 0;
    lastRightOutput_ = 0;

    // Set output limits
    lateralPID_.setOutputLimits(-maxSpeed, maxSpeed);

    float targetTicks = distance * drivetrain_.ticksPerInch();
    float startTime = getCurrentTime();
    float elapsed = 0;
    const int dt = 20;

    // Set up motion profile if enabled
    TrajectoryPoint targetPoint;
    if (useMotionProfile_) {
        lateralProfile_.reset();
        lateralProfile_.generate(targetTicks);
    }

    while (!lateralPID_.isSettled() && elapsed < timeout) {
        float currentPos = drivetrain_.getAveragePosition();
        float output = 0;

        if (useMotionProfile_) {
            // Follow motion profile
            float profileTime = elapsed / 1000.0f; // Convert to seconds
            targetPoint = lateralProfile_.calculate(profileTime);

            // Position PID on profile position
            float pidOutput = lateralPID_.compute(targetPoint.position, currentPos);

            // Scale PID output by max speed
            pidOutput = pidOutput * (maxSpeed / 127.0f);

            // Add feedforward: velocity + acceleration + static friction
            double ff = calculateFeedforward(targetPoint.velocity, targetPoint.acceleration,
                                            lateralkV_, lateralkA_, lateralkS_);

            // Convert feedforward from velocity units to voltage
            // Assuming velocity is in ticks/sec, and 600 RPM motor is ~100 ticks/sec at max
            double ffVoltage = ff * (127.0 / 100.0); // Scale appropriately

            output = pidOutput + ffVoltage;
        } else {
            // Standard PID
            output = lateralPID_.compute(targetTicks, currentPos);
        }

        // Apply slew rate limiting
        output = applySlewRate(output, lastLeftOutput_);

        // Apply to both sides
        if (drivetrain_.leftMotors && drivetrain_.rightMotors) {
            drivetrain_.leftMotors->move(output);
            drivetrain_.rightMotors->move(output);
        }

        pros::delay(dt);
        elapsed = getCurrentTime() - startTime;
    }

    stop();
    return lateralPID_.isSettled();
}

bool Chassis::drive(float distance, float maxSpeed, int timeout) {
    if (maxSpeed < 0) maxSpeed = maxLateralSpeed_;
    if (timeout < 0) timeout = defaultTimeout_;

    return executeDrive(distance, maxSpeed, timeout);
}

void Chassis::driveAsync(float distance, float maxSpeed, int timeout) {
    if (maxSpeed < 0) maxSpeed = maxLateralSpeed_;
    if (timeout < 0) timeout = defaultTimeout_;

    currentMotion_ = MotionType::DRIVE;
    motionSettled_ = false;
    motionTarget_ = distance;
    motionMaxSpeed_ = maxSpeed;
    motionTimeout_ = timeout;
    motionStartTime_ = getCurrentTime();
}

bool Chassis::executeTurn(float angle, float maxSpeed, int timeout) {
    // Reset PID and encoders
    turnPID_.reset();
    drivetrain_.tarePosition();
    lastLeftOutput_ = 0;
    lastRightOutput_ = 0;

    // Set output limits
    turnPID_.setOutputLimits(-maxSpeed, maxSpeed);

    // Calculate approximate tick difference for the turn
    float trackWidth = 12.0f; // inches, adjust for your robot
    float wheelCircumference = drivetrain_.wheelDiameter * static_cast<float>(M_PI);
    float rotationCircumference = trackWidth * static_cast<float>(M_PI);
    float targetTickDiff = (angle / 360.0f) * rotationCircumference * drivetrain_.ticksPerInch();

    float startTime = getCurrentTime();
    float elapsed = 0;
    const int dt = 20;

    // Set up motion profile if enabled
    TrajectoryPoint targetPoint;
    if (useMotionProfile_) {
        turnProfile_.reset();
        turnProfile_.generate(targetTickDiff);
    }

    while (!turnPID_.isSettled() && elapsed < timeout) {
        float currentDiff = drivetrain_.getPositionDifference();
        float output = 0;

        if (useMotionProfile_) {
            float profileTime = elapsed / 1000.0f;
            targetPoint = turnProfile_.calculate(profileTime);

            float pidOutput = turnPID_.compute(targetPoint.position, currentDiff);
            pidOutput = pidOutput * (maxSpeed / 127.0f);

            double ff = calculateFeedforward(targetPoint.velocity, targetPoint.acceleration,
                                            turnkV_, turnkA_, turnkS_);
            double ffVoltage = ff * (127.0 / 100.0);

            output = pidOutput + ffVoltage;
        } else {
            output = turnPID_.compute(targetTickDiff, currentDiff);
        }

        output = applySlewRate(output, lastLeftOutput_);

        if (drivetrain_.leftMotors && drivetrain_.rightMotors) {
            drivetrain_.leftMotors->move(output);
            drivetrain_.rightMotors->move(-output);
        }

        pros::delay(dt);
        elapsed = getCurrentTime() - startTime;
    }

    stop();
    return turnPID_.isSettled();
}

bool Chassis::turn(float angle, float maxSpeed, int timeout) {
    if (maxSpeed < 0) maxSpeed = maxTurnSpeed_;
    if (timeout < 0) timeout = defaultTimeout_;

    return executeTurn(angle, maxSpeed, timeout);
}

void Chassis::turnAsync(float angle, float maxSpeed, int timeout) {
    if (maxSpeed < 0) maxSpeed = maxTurnSpeed_;
    if (timeout < 0) timeout = defaultTimeout_;

    currentMotion_ = MotionType::TURN;
    motionSettled_ = false;
    motionTarget_ = angle;
    motionMaxSpeed_ = maxSpeed;
    motionTimeout_ = timeout;
    motionStartTime_ = getCurrentTime();
}

bool Chassis::executeTurnTo(float heading, float maxSpeed, int timeout) {
    if (!imu_) {
        return turn(heading, maxSpeed, timeout);
    }

    // Reset PID
    turnPID_.reset();
    lastLeftOutput_ = 0;
    lastRightOutput_ = 0;

    // Set output limits
    turnPID_.setOutputLimits(-maxSpeed, maxSpeed);

    // Get current heading and calculate shortest path
    float currentHeading = imu_->get_heading();
    float error = heading - currentHeading;
    while (error > 180) error -= 360;
    while (error < -180) error += 360;
    float targetHeading = currentHeading + error;

    float startTime = getCurrentTime();
    float elapsed = 0;
    const int dt = 20;

    while (!turnPID_.isSettled() && elapsed < timeout) {
        currentHeading = imu_->get_heading();

        // Calculate error with wraparound handling
        error = targetHeading - currentHeading;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        // Compute output
        float output = turnPID_.compute(error, 0);
        output = applySlewRate(output, lastLeftOutput_);

        if (drivetrain_.leftMotors && drivetrain_.rightMotors) {
            drivetrain_.leftMotors->move(output);
            drivetrain_.rightMotors->move(-output);
        }

        pros::delay(dt);
        elapsed = getCurrentTime() - startTime;
    }

    stop();
    return turnPID_.isSettled();
}

bool Chassis::turnTo(float heading, float maxSpeed, int timeout) {
    if (maxSpeed < 0) maxSpeed = maxTurnSpeed_;
    if (timeout < 0) timeout = defaultTimeout_;

    return executeTurnTo(heading, maxSpeed, timeout);
}

void Chassis::turnToAsync(float heading, float maxSpeed, int timeout) {
    if (maxSpeed < 0) maxSpeed = maxTurnSpeed_;
    if (timeout < 0) timeout = defaultTimeout_;

    currentMotion_ = MotionType::TURN_TO;
    motionSettled_ = false;
    motionTarget_ = heading;
    motionMaxSpeed_ = maxSpeed;
    motionTimeout_ = timeout;
    motionStartTime_ = getCurrentTime();
}

bool Chassis::isSettled() const {
    return motionSettled_;
}

bool Chassis::waitUntilSettled(int timeout) {
    if (timeout < 0) timeout = defaultTimeout_;

    float startTime = getCurrentTime();
    float elapsed = 0;
    const int dt = 20;

    while (!motionSettled_ && elapsed < timeout) {
        // Update async motion if needed
        if (currentMotion_ != MotionType::NONE) {
            float currentTime = getCurrentTime();
            float motionElapsed = currentTime - motionStartTime_;

            switch (currentMotion_) {
                case MotionType::DRIVE:
                    motionSettled_ = executeDrive(motionTarget_, motionMaxSpeed_, motionTimeout_);
                    break;
                case MotionType::TURN:
                    motionSettled_ = executeTurn(motionTarget_, motionMaxSpeed_, motionTimeout_);
                    break;
                case MotionType::TURN_TO:
                    motionSettled_ = executeTurnTo(motionTarget_, motionMaxSpeed_, motionTimeout_);
                    break;
                default:
                    break;
            }

            if (motionSettled_) {
                currentMotion_ = MotionType::NONE;
            }
        }

        pros::delay(dt);
        elapsed = getCurrentTime() - startTime;
    }

    return motionSettled_;
}

void Chassis::stopMotion() {
    currentMotion_ = MotionType::NONE;
    motionSettled_ = true;
    stop();
}

bool Chassis::waitForSettle(PIDController& pid, int timeout) {
    int elapsed = 0;
    const int dt = 20;

    while (!pid.isSettled() && elapsed < timeout) {
        if (odometry_) {
            odometry_->update();
        }
        pros::delay(dt);
        elapsed += dt;
    }

    return pid.isSettled();
}

void Chassis::stop(pros::motor_brake_mode_e_t brakeMode) {
    lastLeftOutput_ = 0;
    lastRightOutput_ = 0;

    if (drivetrain_.leftMotors) {
        drivetrain_.leftMotors->move(0);
        drivetrain_.leftMotors->set_brake_modes(brakeMode);
    }
    if (drivetrain_.rightMotors) {
        drivetrain_.rightMotors->move(0);
        drivetrain_.rightMotors->set_brake_modes(brakeMode);
    }
}

void Chassis::tank(int leftVoltage, int rightVoltage) {
    // Clamp to valid range
    if (leftVoltage > 127) leftVoltage = 127;
    if (leftVoltage < -127) leftVoltage = -127;
    if (rightVoltage > 127) rightVoltage = 127;
    if (rightVoltage < -127) rightVoltage = -127;

    // Apply slew rate limiting
    double leftOut = applySlewRate(leftVoltage, lastLeftOutput_);
    double rightOut = applySlewRate(rightVoltage, lastRightOutput_);

    if (drivetrain_.leftMotors) drivetrain_.leftMotors->move(leftOut);
    if (drivetrain_.rightMotors) drivetrain_.rightMotors->move(rightOut);
}

void Chassis::arcade(int forward, int turn) {
    // Clamp inputs
    if (forward > 127) forward = 127;
    if (forward < -127) forward = -127;
    if (turn > 127) turn = 127;
    if (turn < -127) turn = -127;

    // Calculate motor values
    int left = forward + turn;
    int right = forward - turn;

    // Normalize if values exceed 127
    int maxVal = left > right ? left : right;
    int minVal = left < right ? left : right;
    int absMax = maxVal > -minVal ? maxVal : -minVal;

    if (absMax > 127) {
        float scale = 127.0f / absMax;
        left = (int)(left * scale);
        right = (int)(right * scale);
    }

    tank(left, right);
}

bool Chassis::driveToPoint(float x, float y, float maxSpeed, int timeout) {
    if (!odometry_) {
        return false;
    }

    if (maxSpeed < 0) maxSpeed = maxLateralSpeed_;
    if (timeout < 0) timeout = defaultTimeout_;

    lateralPID_.reset();
    lateralPID_.setOutputLimits(-maxSpeed, maxSpeed);
    lastLeftOutput_ = 0;
    lastRightOutput_ = 0;

    PIDController headingPID(navConfig_.headingPID.kP, navConfig_.headingPID.kI, navConfig_.headingPID.kD);
    headingPID.reset();
    headingPID.setOutputLimits(-navConfig_.headingCorrectionLimit, navConfig_.headingCorrectionLimit);

    Pose target(x, y, 0);
    float startTime = getCurrentTime();
    float elapsed = 0;
    const int dt = 20;

    while (elapsed < timeout) {
        Pose current = odometry_->getPose();

        float distance = current.distanceTo(target);
        if (distance < navConfig_.arrivalThreshold && lateralPID_.isSettled()) {
            break;
        }

        float angleToTarget = current.angleTo(target);
        float headingError = Odometry::angleDifference(angleToTarget, current.theta);

        float distanceOutput = lateralPID_.compute(-distance, 0);
        float headingOutput = headingPID.compute(-headingError, 0);

        int leftSpeed = (int)(distanceOutput - headingOutput);
        int rightSpeed = (int)(distanceOutput + headingOutput);

        if (leftSpeed > maxSpeed) leftSpeed = (int)maxSpeed;
        if (leftSpeed < -maxSpeed) leftSpeed = (int)-maxSpeed;
        if (rightSpeed > maxSpeed) rightSpeed = (int)maxSpeed;
        if (rightSpeed < -maxSpeed) rightSpeed = (int)-maxSpeed;

        // Apply slew rate
        double leftOut = applySlewRate(leftSpeed, lastLeftOutput_);
        double rightOut = applySlewRate(rightSpeed, lastRightOutput_);

        if (drivetrain_.leftMotors && drivetrain_.rightMotors) {
            tank((int)leftOut, (int)rightOut);
        }

        pros::delay(dt);
        elapsed = getCurrentTime() - startTime;
    }

    stop();
    return (elapsed < timeout);
}

bool Chassis::turnToPoint(float x, float y, float maxSpeed, int timeout) {
    if (!odometry_) {
        return false;
    }

    if (maxSpeed < 0) maxSpeed = maxTurnSpeed_;
    if (timeout < 0) timeout = defaultTimeout_;

    turnPID_.reset();
    turnPID_.setOutputLimits(-maxSpeed, maxSpeed);
    turnPID_.setTolerance(navConfig_.turnThreshold, navConfig_.turnSettleSamples);
    lastLeftOutput_ = 0;
    lastRightOutput_ = 0;

    Pose target(x, y, 0);
    float startTime = getCurrentTime();
    float elapsed = 0;
    const int dt = 20;

    while (!turnPID_.isSettled() && elapsed < timeout) {
        Pose current = odometry_->getPose();

        float angleToTarget = current.angleTo(target);
        float error = Odometry::angleDifference(angleToTarget, current.theta);

        float output = turnPID_.compute(error, 0);
        output = applySlewRate(output, lastLeftOutput_);

        if (drivetrain_.leftMotors && drivetrain_.rightMotors) {
            drivetrain_.leftMotors->move(output);
            drivetrain_.rightMotors->move(-output);
        }

        pros::delay(dt);
        elapsed = getCurrentTime() - startTime;
    }

    stop();
    return turnPID_.isSettled();
}

Pose Chassis::getPose() const {
    if (odometry_) {
        return odometry_->getPose();
    }
    return Pose(0, 0, 0);
}

void Chassis::setPose(const Pose& pose) {
    if (odometry_) {
        odometry_->setPose(pose);
    }
}

void Chassis::setOdometry(Odometry* odom) {
    odometry_ = odom;
}

} // namespace Helix
