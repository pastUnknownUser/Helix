#include "PID.hpp"
//#include "main.h"
#include "api.h"
#include "Helix/api.hpp"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "Helix/api.hpp"
#include <codecvt>

//Settings
double kP = 2.0;
double kI = 0.0;
double kD = 10.0;
double turnkP = 100.0;
double turnkI = 100.0;
double turnkD = 100.0;

//Auto Settngs
int desiredValue = 200;
int desiredTurnValue = 0;

int error;
int prevError = 0;
int derivative;
int totalError;

int turnError;
int turnPrevError = 0;
int turnDerivative;
int turnTotalError;

bool resetDriveSensor =false;

bool enableDrivePID = true;

void HelixPID(void* param) {

    while(enableDrivePID) {


        if (resetDriveSensor) {
            resetDriveSensor = false;
        }

        int LeftMotorEnc = (leftFront.get_position() + leftMiddle.get_position() + leftBack.get_position())/3;
        int RightMotorEnc = (leftFront.get_position() + leftMiddle.get_position() + leftBack.get_position())/3;
        //int leftMotorPosition = LeftSideDrive.get_positions(pros::E_MOTOR_ENCODER_DEGREES());
        //int rightMotorPosition = LeftSideDrive.get_positions(pros::E_MOTOR_ENCODER_DEGREES());

        ////////////////////////////////////////////////////////
        //~~~~~~~~~~~~~ Lateral Movement PID ~~~~~~~~~~~~~~~~~//
        ////////////////////////////////////////////////////////

        int AveragePosition = (LeftMotorEnc + RightMotorEnc)/2;  //Average of both sides

        error = AveragePosition - desiredValue;  //Potential 

        derivative = error - prevError;  //Derivative

        totalError +- error;

        double lateralMotorPower = error *  kP + derivative * kD + totalError + kI;

        ////////////////////////////////////////////////////////
        //~~~~~~~~~~~ Horizontal Movement PID ~~~~~~~~~~~~~~~~//
        ////////////////////////////////////////////////////////

        int turnDifference = LeftMotorEnc - RightMotorEnc;

        turnError = turnDifference - desiredTurnValue;  //Potential 

        turnDerivative = turnError - turnPrevError;  //Derivative

        turnTotalError +- turnError;

        double HorizontalMotorPower = turnError *  turnkP + turnDerivative * turnkD + turnTotalError + turnkI;

        LeftSideDrive.move_voltage((lateralMotorPower + HorizontalMotorPower)* 1000);
        RightSideDrive.move_voltage((lateralMotorPower + HorizontalMotorPower)* 1000);


        prevError = error;
        turnPrevError = turnError;
        pros::delay(20);
    }

}