Chassis Configuration
=====================

The ``Helix::Chassis`` class manages the physical execution of linear and angular movements. It couples target linear inch requests directly to our integrated PID math module.

Source implementation inside ``chassis.cpp``:

.. code-block:: cpp

    #include "Helix/helixApi.h"
    #include "pros/motor_group.hpp"

    pros::Imu imu(14);
    Helix::Chassis chassis(leftSide, rightSide, imu, 3.25, 1.33333);

Method Overview
---------------

.. cpp:function:: void Chassis::setDrivePID(double p, double i, double d, double max_i, double max_out, double tol)

   Configures the lateral PID constants.

.. cpp:function:: void Chassis::move(double targetInches)

   Executes a straight-line movement using encoder calculations. Automatically resets position and tracks output until target is settled.

.. cpp:function:: void Chassis::turn(double targetDegrees)

   Uses the inertial sensor to turn to a requested heading with a bounded PID loop.
