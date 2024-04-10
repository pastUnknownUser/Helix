#include "PID.hpp"
#include "main.h"

//Settings
double kP = 0.0;
double kI = 0.0;
double kD = 0.0;

//Auto Settngs
int desiredValue = 200;

int error;
int prevError = 0;
int derivative;
int totalError;

bool enableDrivePID = true;

void FPID() {

    while(enableDrivePID) {

        int LeftSidePosition = ();


    }

}