/**
 * @file main.cpp
 * @brief Example implementation of Helix PID library with advanced features
 *
 * This file demonstrates how to use the Helix library with:
 * - Motion profiling (smooth acceleration/deceleration)
 * - Feedforward control (kV, kA, kS)
 * - Async movements (non-blocking)
 * - Slew rate limiting
 * - SD Card telemetry logging (when available)
 */

#include "main.h"
#include "Helix/Helix.hpp"
#include <optional>

// ============================================================
// MOTOR SETUP
// ============================================================

// Left side motors (reversed = true for tank drive)
pros::Motor leftFront(11, pros::E_MOTOR_GEAR_BLUE, true);
pros::Motor leftMiddle(3, pros::E_MOTOR_GEAR_BLUE, true);
pros::Motor leftBack(1, pros::E_MOTOR_GEAR_BLUE, true);

// Right side motors (reversed = false)
pros::Motor rightFront(17, pros::E_MOTOR_GEAR_BLUE, false);
pros::Motor rightMiddle(10, pros::E_MOTOR_GEAR_BLUE, false);
pros::Motor rightBack(8, pros::E_MOTOR_GEAR_BLUE, false);

// Group motors for easier control
pros::Motor_Group leftSideDrive({leftFront, leftMiddle, leftBack});
pros::Motor_Group rightSideDrive({rightFront, rightMiddle, rightBack});

// IMU for accurate turning and heading
pros::IMU imu(19);

// ============================================================
// HELIX CONFIGURATION
// ============================================================

std::optional<Helix::Odometry> odom;
std::optional<Helix::Chassis> chassis;

// Logger for telemetry with SD card support
Helix::Logger logger;

// ============================================================
// CONTROLLER SETUP
// ============================================================

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// ============================================================
// COMPETITION FUNCTIONS
// ============================================================

void initialize() {
    // Initialize LCD
    pros::lcd::initialize();
    pros::lcd::set_text(0, "Helix Advanced Demo");
    pros::lcd::set_text(1, "Calibrating IMU...");

    // Set up logger and check for SD card
    logger.setLevel(Helix::LogLevel::INFO);
    if (logger.initSDCard()) {
        pros::lcd::set_text(2, "SD Card: Yes");
    } else {
        pros::lcd::set_text(2, "SD Card: No");
    }
    logger.info("Initializing Helix...");

    // Calibrate IMU
    imu.reset();
    int calibrationTimeout = 0;
    const int maxCalibrationTime = 5000;
    while (imu.is_calibrating() && calibrationTimeout < maxCalibrationTime) {
        pros::delay(10);
        calibrationTimeout += 10;
    }

    if (calibrationTimeout >= maxCalibrationTime) {
        pros::lcd::set_text(1, "IMU Cal Timeout!");
        logger.warn("IMU calibration timed out");
    } else {
        pros::lcd::set_text(1, "IMU Ready!");
        logger.info("IMU calibration complete");
    }

    // Set brake modes
    leftSideDrive.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
    rightSideDrive.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);

    // ============================================================
    // ODOMETRY CONFIGURATION
    // ============================================================
    Helix::Odometry::Config odomConfig;
    odomConfig.leftMotors = &leftSideDrive;
    odomConfig.rightMotors = &rightSideDrive;
    odomConfig.imu = &imu;
    odomConfig.wheelDiameter = 3.25f;
    odomConfig.trackWidth = 12.0f;

    odom.emplace(odomConfig);

    // ============================================================
    // CHASSIS CONFIGURATION - ADVANCED
    // ============================================================
    Helix::Chassis::Config chassisConfig;
    chassisConfig.drivetrain = Helix::Drivetrain(
        &leftSideDrive, &rightSideDrive, 600, 3.25
    );

    // PID configuration
    chassisConfig.lateralPID = Helix::PIDController(0.8, 0.001, 0.1);
    chassisConfig.turnPID = Helix::PIDController(1.5, 0.002, 0.15);
    chassisConfig.imu = &imu;

    // Speed limits
    chassisConfig.maxLateralSpeed = 100;
    chassisConfig.maxTurnSpeed = 80;
    chassisConfig.defaultTimeout = 3000;

    // Feedforward gains (tune for your robot)
    chassisConfig.lateralkV = 0.12;
    chassisConfig.lateralkA = 0.002;
    chassisConfig.lateralkS = 3.0;
    chassisConfig.turnkV = 0.08;
    chassisConfig.turnkA = 0.001;
    chassisConfig.turnkS = 2.0;

    // Motion profiling
    chassisConfig.useMotionProfile = true;
    chassisConfig.maxJerk = 40.0;
    chassisConfig.maxAcceleration = 16.0;
    chassisConfig.maxVelocity = 60.0;

    // Slew rate limiting (volts/sec)
    chassisConfig.slewRate = 600.0;

    // Create chassis
    chassis.emplace(chassisConfig);
    chassis->setOdometry(&*odom);

    pros::lcd::set_text(3, "Ready!");
    logger.info("Helix initialization complete");
}

void disabled() {}

void competition_initialize() {
    pros::lcd::set_text(4, "Select Auton Mode");
}

void autonomous() {
    // Reset odometry
    odom->setPose(Helix::Pose(0, 0, 0));

    // Start recording telemetry
    logger.startRecording();
    logger.info("Autonomous started");

    // ============================================================
    // EXAMPLE 1: Basic movements with motion profiling
    // ============================================================
    chassis->drive(24);      // 24 inches forward
    pros::delay(200);
    chassis->turn(90);       // Turn 90 degrees
    pros::delay(200);
    chassis->drive(24);      // Drive forward
    pros::delay(200);
    chassis->turnTo(0);      // Face heading 0

    // ============================================================
    // EXAMPLE 2: Async movements
    // ============================================================
    odom->setPose(Helix::Pose(0, 0, 0));

    // Start async drive
    chassis->driveAsync(36, 80, 4000);
    pros::delay(500);

    // Check status
    if (!chassis->isSettled()) {
        logger.info("Drive in progress...");
    }

    // Wait for completion
    chassis->waitUntilSettled();
    logger.info("Drive complete!");

    // ============================================================
    // EXAMPLE 3: Point-to-point navigation
    // ============================================================
    odom->setPose(Helix::Pose(0, 0, 90));

    chassis->driveToPoint(24, 24, 80, 5000);
    pros::delay(200);
    chassis->turnToPoint(0, 24);
    pros::delay(200);
    chassis->driveToPoint(0, 24, 80, 5000);
    pros::delay(200);
    chassis->driveToPoint(0, 0, 60, 5000);

    // Stop recording
    logger.stopRecording();
    logger.info("Autonomous complete!");
}

void opcontrol() {
    pros::lcd::clear_line(4);
    pros::lcd::set_text(4, "Driver Control");
    logger.info("Driver control started");

    int loopCount = 0;

    while (true) {
        // Update odometry
        odom->update();

        // Get current pose
        Helix::Pose pose = odom->getPose();

        // Display on LCD
        if (loopCount % 10 == 0) {
            pros::lcd::print(5, "X: %.1f Y: %.1f H: %.1f", pose.x, pose.y, pose.theta);
        }
        loopCount++;

        // Get joystick inputs
        int forward = controller.get_analog(ANALOG_LEFT_Y);
        int turn = controller.get_analog(ANALOG_RIGHT_X);

        // Arcade drive with slew rate limiting
        chassis->arcade(forward, turn);

        // Reset odometry
        if (controller.get_digital_new_press(DIGITAL_Y)) {
            odom->setPose(Helix::Pose(0, 0, 0));
            pros::lcd::set_text(7, "Odom Reset!");
            logger.info("Odometry reset");
        }

        // Async drive test
        if (controller.get_digital_new_press(DIGITAL_A)) {
            chassis->stop();
            logger.info("Starting async drive");
            chassis->driveAsync(24, 80, 3000);
        }

        // Complete async motion
        if (controller.get_digital_new_press(DIGITAL_B)) {
            if (!chassis->isSettled()) {
                logger.info("Completing async motion...");
                chassis->waitUntilSettled();
                logger.info("Async motion complete");
            }
        }

        // Turn to point
        if (controller.get_digital_new_press(DIGITAL_X)) {
            chassis->stop();
            logger.info("Turning to face (24, 24)");
            chassis->turnToPoint(24, 24);
        }

        // Emergency stop
        if (controller.get_digital(DIGITAL_L1)) {
            chassis->stop();
            chassis->stopMotion();
        }

        // Start/stop logging with R1
        static bool logging = false;
        if (controller.get_digital_new_press(DIGITAL_R1)) {
            if (!logging) {
                logger.startRecording();
                logging = true;
                pros::lcd::set_text(6, "Logging: ON");
            } else {
                logger.stopRecording();
                logging = false;
                pros::lcd::set_text(6, "Logging: OFF");
            }
        }

        pros::delay(20);
    }
}
