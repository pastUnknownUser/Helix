#include "main.h"
#include "PID.hpp"
#include "Helix/api.hpp"
#include "Helix/chassisConfig.hpp"

pros::Motor leftBack(5);
pros::Motor leftMiddle(5);
pros::Motor leftFront(5);
pros::Motor rightBack(10,false);
pros::Motor rightMiddle(5,false);
pros::Motor rightFront(5,false);
pros::Motor_Group LeftSideDrive({leftBack, leftMiddle, leftFront});
pros::Motor_Group RightSideDrive({rightBack, rightMiddle, rightFront});

pros::IMU Inertial(19); // port 19

int main()
{
    Helix::Test me;
    me.leftMotors = &LeftSideDrive;
    me.rightMotors = &RightSideDrive;
    me.trackWidth = 10;
    me.wheelDiameter = 3.25;
    me.rpm = 360;
    me.chasePower = 3;
}
/*
Helix::Drivetrain drivetrain {
    &LeftSideDrive, // left drivetrain motors
    &RightSideDrive, // right drivetrain motors
    10, // track width
    3.25, // wheel diameter
    360, // wheel rpm
	5 //Chase Power
};

Helix::Sensors sensors {
    &Inertial
};

Helix::Chassis chassis(drivetrain, sensors);
*/