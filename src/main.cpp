/**
 * @file main.cpp
 * @brief Example implementation of Helix PID library with Odometry
 *
 * This file demonstrates how to set up and use the Helix library
 * for autonomous robot control with position tracking.
 */

#include "main.h"
#include "Helix/Helix.hpp"

// ============================================================
// MOTOR SETUP
// ============================================================
// Configure your motors here. Change gear ratios and reversals as needed.

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

// IMU for accurate turning and heading (required for odometry)
pros::IMU imu(19);

// ============================================================
// HELIX CONFIGURATION
// ============================================================

// Declare pointers - initialized in initialize()
Helix::Chassis* chassis = nullptr;
Helix::Odometry* odom = nullptr;

// ============================================================
// CONTROLLER SETUP
// ============================================================

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// ============================================================
// COMPETITION FUNCTIONS
// ============================================================

/**
 * @brief Runs initialization code when the program starts
 *
 * Calibrate IMU, set brake modes, initialize odometry, etc.
 */
void initialize() {
    // Initialize LCD for debugging
    pros::lcd::initialize();
    pros::lcd::set_text(0, "Helix PID + Odom");
    pros::lcd::set_text(1, "Calibrating IMU...");

    // Calibrate IMU (takes ~2 seconds)
    imu.reset();
    while (imu.is_calibrating()) {
        pros::delay(10);
    }
    pros::lcd::set_text(1, "IMU Ready!");

    // Set brake modes (optional)
    leftSideDrive.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
    rightSideDrive.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);

    // Create odometry configuration
    Helix::Odometry::Config odomConfig;
    odomConfig.leftMotors = &leftSideDrive;
    odomConfig.rightMotors = &rightSideDrive;
    odomConfig.imu = &imu;
    odomConfig.wheelDiameter = 3.25f;
    odomConfig.trackWidth = 12.0f;      // Adjust to your robot's track width
    odomConfig.rpm = 600.0f;

    // Create odometry
    odom = new Helix::Odometry(odomConfig);

    // Create chassis configuration
    Helix::Chassis::Config chassisConfig;
    chassisConfig.drivetrain = Helix::Drivetrain(
        &leftSideDrive,
        &rightSideDrive,
        600,
        3.25
    );
    chassisConfig.lateralPID = Helix::PIDController(0.8, 0.001, 0.1);
    chassisConfig.turnPID = Helix::PIDController(1.5, 0.002, 0.15);
    chassisConfig.imu = &imu;
    chassisConfig.maxLateralSpeed = 100;
    chassisConfig.maxTurnSpeed = 80;
    chassisConfig.defaultTimeout = 2000;

    // Create chassis
    chassis = new Helix::Chassis(chassisConfig);

    // Link odometry to chassis
    chassis->setOdometry(odom);

    pros::lcd::set_text(2, "Ready!");
}

/**
 * @brief Runs while the robot is disabled
 */
void disabled() {
    // Nothing to do here
}

/**
 * @brief Runs after initialize(), before autonomous
 * Use for competition-specific setup (e.g., autonomous selector)
 */
void competition_initialize() {
    // Example: Display autonomous selector on LCD
    pros::lcd::set_text(3, "Select Auton Mode");
}

/**
 * @brief Autonomous routine with odometry
 *
 * Uses field coordinates to navigate. (0,0) is starting position.
 * X = forward/back (positive = forward)
 * Y = left/right (positive = right)
 * Theta = heading (0 = forward, 90 = right, etc.)
 */
void autonomous() {
    // Reset to starting position
    odom->setPose(Helix::Pose(0, 0, 0));

    // ============================================================
    // EXAMPLE 1: Drive to specific coordinates
    // ============================================================
    // Drive forward 24 inches (X = 24, Y = 0)
    chassis->driveToPoint(24, 0);

    // Drive to a point diagonally right (X = 36, Y = 24)
    chassis->driveToPoint(36, 24);

    // Turn to face the origin
    chassis->turnToPoint(0, 0);

    // Drive back to start
    chassis->driveToPoint(0, 0);

    // Turn to original heading
    chassis->turnTo(0);

    // ============================================================
    // EXAMPLE 2: Square pattern using coordinates
    // ============================================================
    // Reset at center of field
    odom->setPose(Helix::Pose(0, 0, 0));

    // Drive in a square
    for (int i = 0; i < 4; i++) {
        // Drive forward 24 inches
        chassis->driveToPoint(
            odom->getPose().x + 24 * std::cos(odom->getPose().theta * 3.14159 / 180),
            odom->getPose().y + 24 * std::sin(odom->getPose().theta * 3.14159 / 180)
        );

        // Turn 90 degrees right
        chassis->turn(90);
    }

    // ============================================================
    // EXAMPLE 3: Traditional movements still work
    // ============================================================
    chassis->drive(24);      // Drive 24 inches forward
    chassis->turn(90);       // Turn 90 degrees right
    chassis->drive(-12);      // Drive 12 inches backward
}

/**
 * @brief Operator control (driver control)
 *
 * Displays odometry data on LCD for debugging
 */
void opcontrol() {
    pros::lcd::clear_line(2);
    pros::lcd::set_text(2, "Driver Control");

    int loopCount = 0;

    while (true) {
        // Update odometry
        odom->update();

        // Get current pose
        Helix::Pose pose = odom->getPose();

        // Display on LCD every 10 loops (to avoid flickering)
        if (loopCount % 10 == 0) {
            pros::lcd::print(4, "X: %.1f Y: %.1f", pose.x, pose.y);
            pros::lcd::print(5, "Heading: %.1f", pose.theta);
            pros::lcd::print(6, "Dist: %.1f", odom->getTotalDistance());
        }
        loopCount++;

        // Get joystick inputs
        int forward = controller.get_analog(ANALOG_LEFT_Y);
        int turn = controller.get_analog(ANALOG_RIGHT_X);

        // Arcade drive using chassis
        chassis->arcade(forward, turn);

        // Reset odometry with button press
        if (controller.get_digital_new_press(DIGITAL_Y)) {
            odom->setPose(Helix::Pose(0, 0, 0));
            pros::lcd::set_text(7, "Odom Reset!");
        }

        // Test drive to point with A button
        if (controller.get_digital_new_press(DIGITAL_A)) {
            chassis->stop();
            chassis->driveToPoint(24, 0, 80, 3000);  // Drive to (24, 0)
        }

        // Test turn to point with B button
        if (controller.get_digital_new_press(DIGITAL_B)) {
            chassis->stop();
            chassis->turnToPoint(0, 24);  // Turn to face +Y
        }

        // Emergency stop with X
        if (controller.get_digital(DIGITAL_X)) {
            chassis->stop();
        }

        pros::delay(20);  // Run at 50Hz
    }
}
