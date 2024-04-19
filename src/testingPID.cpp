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

// Global variables for PID controller
double integral = 0;
double previous_error = 0;

int chassisRPM = 360;
double LeftPose = (leftBack.get_position() + leftMiddle.get_position() + leftFront.get_position()/3);
double RightPose = (rightBack.get_position() + rightMiddle.get_position() + rightFront.get_position()/3);

void moveRobot(double distance) {
    double target_rotations = distance / (3.25 * M_PI); // Convert inches to rotations
    double left_current_rotations = LeftPose / chassisRPM;
    double right_current_rotations = RightPose / chassisRPM;

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
