#include "PID.hpp"
//#include "main.h"
#include "api.h"
#include "Helix/api.hpp"
#include "pros/rtos.h"
#include "Helix/api.hpp"
#include <codecvt>
#include <cmath>
#include "main.h"

// Constants for the PID controller
const double kP = 0.1;
const double kI = 0.01;
const double kD = 0.001;

// Function prototypes
double calculatePID(double target, double current);
void moveRobot(double distance);

const double wheelDiameter = 3.25; // Diameter of the wheel in inches
const double rpm = 360.0; // Encoder counts per revolution
const double Inches = rpm / (wheelDiameter * 3.14159); // Calculate encoder counts per inch

// Global variables for PID controller
double integral = 0;
double previous_error = 0;

int chassisRPM = 360;

void moveRobot(double distance, bool Reverse) {
    double target_rotations = distance / Inches; // Convert inches to rotations
    double left_current_rotations = leftFront.get_position() / chassisRPM;
    double right_current_rotations = rightFront.get_position() / chassisRPM;

    double left_error = target_rotations - left_current_rotations;
    double right_error = target_rotations - right_current_rotations;

    double left_speed = calculatePID(target_rotations, left_current_rotations);
    double right_speed = calculatePID(target_rotations, right_current_rotations);

    LeftSideDrive.move_voltage(left_speed * 12000); // Assuming 120V max voltage
    RightSideDrive.move_voltage(right_speed * 12000);
}

double calculatePID(double target, double current) {
    double error = target - current;
    integral += error;
    double derivative = error - previous_error;
    double output = kP * error + kI * integral + kD * derivative;
    previous_error = error;
    return output;
}
