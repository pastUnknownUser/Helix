/**
 * @file main.cpp
 * @brief Example implementation of Helix PID library
 *
 * This file demonstrates how to set up and use the Helix library
 * for autonomous robot control.
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

// IMU for accurate turning (optional but recommended)
// Port 19, calibrate in initialize()
pros::IMU imu(19);

// ============================================================
// HELIX CHASSIS CONFIGURATION
// ============================================================

// Configure the drivetrain physical parameters
Helix::Drivetrain drivetrain(
    &leftSideDrive,   // Left motor group pointer
    &rightSideDrive,  // Right motor group pointer
    600,               // Motor RPM (blue = 600, green = 200, red = 100)
    3.25               // Wheel diameter in inches
);

// Configure PID controllers
// Tuning tips:
// 1. Start with kP only - increase until robot oscillates
// 2. Add kD to reduce oscillation (try kP/10)
// 3. Add kI only if there's steady-state error (start very small)
Helix::PIDController lateralPID(0.8, 0.001, 0.1);  // Drive straight
Helix::PIDController turnPID(1.5, 0.002, 0.15);       // Turn in place

// Assemble chassis configuration
Helix::Chassis::Config config;
config.drivetrain = drivetrain;
config.lateralPID = lateralPID;
config.turnPID = turnPID;
config.imu = &imu;                          // Optional - enables turnTo()
config.maxLateralSpeed = 100;               // Limit drive speed (0-127)
config.maxTurnSpeed = 80;                   // Limit turn speed (0-127)
config.defaultTimeout = 2000;               // Default 2 second timeout

// Create the chassis controller
Helix::Chassis chassis(config);

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
 * Calibrate IMU, set brake modes, etc.
 */
void initialize() {
    // Initialize LCD for debugging
    pros::lcd::initialize();
    pros::lcd::set_text(0, "Helix PID Template");
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
    pros::lcd::set_text(2, "Select Auton Mode");
}

/**
 * @brief Autonomous routine
 *
 * This is where you program your autonomous movements.
 * Each function blocks until complete or timed out.
 */
void autonomous() {
    // Example 1: Basic movements
    // Drive forward 24 inches
    chassis.drive(24);

    // Turn 90 degrees right
    chassis.turn(90);

    // Drive backward 12 inches at 50% speed
    chassis.drive(-12, 60);

    // Turn to absolute heading (requires IMU)
    // 0 = starting orientation, 90 = right, 180 = back, 270 = left
    chassis.turnTo(0);

    // Example 2: Continuous movements with custom timeout
    // Drive 48 inches with 5 second timeout
    bool success = chassis.drive(48, 100, 5000);
    if (!success) {
        // Movement timed out
        pros::lcd::set_text(3, "Drive timed out!");
    }

    // Example 3: PID tuning during runtime
    // If robot is oscillating, reduce kD
    // If robot stops short, increase kP or add kI
    chassis.getLateralPID().kP = 1.0;  // Adjust as needed

    // Example 4: Complex path
    chassis.drive(36);      // Forward 36"
    chassis.turn(45);       // Turn 45° right
    chassis.drive(12);      // Forward 12"
    chassis.turn(-45);      // Turn 45° left (back to original heading)
    chassis.drive(-36);     // Backward 36" to start
}

/**
 * @brief Operator control (driver control)
 *
 * Use the chassis for arcade or tank drive control
 */
void opcontrol() {
    pros::lcd::clear_line(2);
    pros::lcd::set_text(2, "Driver Control");

    while (true) {
        // Get joystick inputs
        int forward = controller.get_analog(ANALOG_LEFT_Y);   // Forward/back
        int turn = controller.get_analog(ANALOG_RIGHT_X);     // Turning

        // Arcade drive using chassis
        chassis.arcade(forward, turn);

        // Alternative: Tank drive
        // int left = controller.get_analog(ANALOG_LEFT_Y);
        // int right = controller.get_analog(ANALOG_RIGHT_Y);
        // chassis.tank(left, right);

        // Alternative: Direct motor control (faster, less overhead)
        // leftSideDrive.move(forward + turn);
        // rightSideDrive.move(forward - turn);

        // Test autonomous movements with button presses
        // A button = test drive forward
        if (controller.get_digital_new_press(DIGITAL_A)) {
            chassis.stop();  // Stop driver control
            chassis.drive(12);  // Drive 12 inches forward
        }

        // B button = test turn
        if (controller.get_digital_new_press(DIGITAL_B)) {
            chassis.stop();
            chassis.turn(90);  // Turn 90 degrees
        }

        // X button = emergency stop
        if (controller.get_digital(DIGITAL_X)) {
            chassis.stop();
        }

        pros::delay(20);  // Run at 50Hz
    }
}
