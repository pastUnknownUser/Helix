#include "PID.hpp"
#include "Nebula/chassisConfig.hpp"
#include "main.h"

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

bool enableDrivePID = true;

void FPID() {

    while(enableDrivePID) {
        
        ////////////////////////////////////////////////////////
        //~~~~~~~~~~~~~Lateral Movement PID ~~~~~~~~~~~~~~~~~~//
        ////////////////////////////////////////////////////////

        int LeftSidePosition = ();  //Left Side Drive
        int RightSidePosition =();  //Right Side Drive

        int AveragePosition = (LeftSidePosition + RightSidePosition)/2;  //Average of both sides

        error = averagePosition - desiredValue  //Potential 

        derivative = error - prevError;  //Derivative

        totalError +- error;

        double lateralMotorPower = turnerror *  kp + Derivative * kD + turntotalError + kI;

        ////////////////////////////////////////////////////////
        //~~~~~~~~~~~~~Turning Movement PID ~~~~~~~~~~~~~~~~~~//
        ////////////////////////////////////////////////////////

        int turnDifference = LeftSidePosition - RightSidePosition;

        turnError = turnDifference - turnDesiredValue  //Potential 

        turnDerivative = turnError - turnPrevError;  //Derivative

        turnTotalError +- turnError;

        double HorizontalMotorPower = turnError *  turnkp + turnDerivative * turnkD + turnTotalError + turnkI;


        prevError = error;
        turnprevError = turnerror;
        PRSO::delay (20);
    }

}