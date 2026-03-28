#pragma once

/**
 * @file Helix.hpp
 * @brief Main include file for the Helix PID library
 *
 * Helix is a simple, configurable PID controller library for PROS VEX V5 robots.
 * It provides easy-to-use PID controllers and a chassis wrapper for autonomous movements.
 *
 * Quick Start Example:
 * @code
 * #include "Helix/Helix.hpp"
 *
 * // Create motors
 * pros::Motor leftFront(1, pros::E_MOTOR_GEAR_BLUE, true);
 * pros::Motor leftBack(2, pros::E_MOTOR_GEAR_BLUE, true);
 * pros::Motor rightFront(3, pros::E_MOTOR_GEAR_BLUE, false);
 * pros::Motor rightBack(4, pros::E_MOTOR_GEAR_BLUE, false);
 *
 * pros::Motor_Group leftSide({leftFront, leftBack});
 * pros::Motor_Group rightSide({rightFront, rightBack});
 *
 * // Configure drivetrain
 * Helix::Drivetrain drivetrain(
 *     &leftSide, &rightSide,  // Motor groups
 *     600,                      // RPM (blue cartridge)
 *     3.25                      // Wheel diameter in inches
 * );
 *
 * // Configure chassis with PID
 * Helix::Chassis::Config config;
 * config.drivetrain = drivetrain;
 * config.lateralPID = Helix::PIDController(0.8, 0.001, 0.1);
 * config.turnPID = Helix::PIDController(1.5, 0.002, 0.2);
 *
 * Helix::Chassis chassis(config);
 *
 * // In autonomous:
 * void autonomous() {
 *     chassis.drive(24);      // Drive forward 24 inches
 *     chassis.turn(90);         // Turn 90 degrees right
 *     chassis.drive(-12);     // Drive backward 12 inches
 * }
 * @endcode
 */

#include "Helix/PIDController.hpp"
#include "Helix/Odometry.hpp"
#include "Helix/Chassis.hpp"

/**
 * @namespace Helix
 * @brief Contains all Helix library components
 */
