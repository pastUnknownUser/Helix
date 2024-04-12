#include "PID.hpp"
//#include "main.h"
#include "api.h"
#include "Helix/api.hpp"
#include "pros/rtos.h"

//Settings
double kP = 0.0;
double kI = 0.0;
double kD = 0.0;
double turnkP = 0.0;
double turnkI = 0.0;
double turnkD = 0.0;

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

void FPID() {

    while(enableDrivePID) {


        if (resetDriveSensor) {
            resetDriveSensor = false;
        }
        
        ////////////////////////////////////////////////////////
        //~~~~~~~~~~~~~ Lateral Movement PID ~~~~~~~~~~~~~~~~~//
        ////////////////////////////////////////////////////////

        int LeftMotors = (0);  //Left Side Drive
        int RightMotors =(0);  //Right Side Drive

        int AveragePosition = (LeftMotors + RightMotors)/2;  //Average of both sides

        error = AveragePosition - desiredValue;  //Potential 

        derivative = error - prevError;  //Derivative

        totalError +- error;

        double lateralMotorPower = error *  kP + derivative * kD + totalError + kI;

        ////////////////////////////////////////////////////////
        //~~~~~~~~~~~ Horizontal Movement PID ~~~~~~~~~~~~~~~~//
        ////////////////////////////////////////////////////////

        int turnDifference = LeftMotors - RightMotors;

        turnError = turnDifference - desiredTurnValue;  //Potential 

        turnDerivative = turnError - turnPrevError;  //Derivative

        turnTotalError +- turnError;

        double HorizontalMotorPower = turnError *  turnkP + turnDerivative * turnkD + turnTotalError + turnkI;

        prevError = error;
        turnPrevError = turnError;
        pros::c::delay(20);
    }

}