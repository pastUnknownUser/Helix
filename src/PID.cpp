#include "PID.hpp"
//#include "main.h"
#include "api.h"
#include "Helix/api.hpp"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "Helix/api.hpp"
#include <codecvt>
#include <cmath>

//Settings
double kP = 10.0;
double kI = 0.0005;
double kD = 10.0;
double turnkP = 100.0;
double turnkI = 100.0;
double turnkD = 100.0;

//Auto Settngs
int desiredValue = 5;
int desiredTurnValue = 0;

int error;
int prevError = 0;
int derivative;
double totalError = 0.0;
double turnTotalError = 0.0;

int turnError;
int turnPrevError = 0;
int turnDerivative;

bool resetDriveSensor =false;
const double wheelDiameter = 3.25; // Diameter of the wheel in inches
const double rpm = 360.0; // Encoder counts per revolution
const double tooInches = rpm / (wheelDiameter * 3.14159); // Calculate encoder counts per inch


/*void HelixPID(void* param) {

    while(enableDrivePID) {


        if (resetDriveSensor) {
            resetDriveSensor = false;
        }

        int LeftMotorEnc = (leftFront.get_position() + leftMiddle.get_position() + leftBack.get_position())/3;
        int RightMotorEnc = (rightFront.get_position() + rightMiddle.get_position() + rightBack.get_position())/3;
        //int leftMotorPosition = LeftSideDrive.get_positions(pros::E_MOTOR_ENCODER_DEGREES());
        //int rightMotorPosition = LeftSideDrive.get_positions(pros::E_MOTOR_ENCODER_DEGREES());

        ////////////////////////////////////////////////////////
        //~~~~~~~~~~~~~ Lateral Movement PID ~~~~~~~~~~~~~~~~~//
        ////////////////////////////////////////////////////////

        int AveragePosition = (LeftMotorEnc + RightMotorEnc)/2;  //Average of both sides

        error = AveragePosition - desiredValue;  //Potential 

        derivative = error - prevError;  //Derivative

        totalError += error * 0.02;

        double lateralMotorPower = error *  kP + derivative * kD + totalError + kI;

        ////////////////////////////////////////////////////////
        //~~~~~~~~~~~ Horizontal Movement PID ~~~~~~~~~~~~~~~~//
        ////////////////////////////////////////////////////////

        int turnDifference = LeftMotorEnc - RightMotorEnc;

        turnError = turnDifference - desiredTurnValue;  //Potential 

        turnDerivative = turnError - turnPrevError;  //Derivative

        turnTotalError +- turnError * 0.02;

        double HorizontalMotorPower = turnError *  turnkP + turnDerivative * turnkD + turnTotalError + turnkI;

        // Print values to the V5 Brain's screen
        pros::lcd::print(0, "Error: %d", error);
        pros::lcd::print(1, "Lateral Power: %.2f", lateralMotorPower);
        pros::lcd::print(2, "Horizontal Power: %.2f", HorizontalMotorPower);

        // Control robot movement using PID outputs
        moveRobot((int) (lateralMotorPower + HorizontalMotorPower), (int) (lateralMotorPower - HorizontalMotorPower));

        prevError = error;
        turnPrevError = turnError;
        pros::delay(20);
    }

}*/