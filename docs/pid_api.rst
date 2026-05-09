PID Control Module
==================

Our system isolates feedback logic into a reusable ``PID`` class. It supports integral windup clamping, cross-zero integral resets, and tolerance checks.

.. code-block:: cpp

    #include "Helix/PID.h"

    // Constructor parameters
    PID::PID(double p, double i, double d);

Feedback Logic Properties
-------------------------

* **Integral Clamping**: Configured using ``setLimits(max_i, max_out)`` to restrict cumulative error buildup.
* **Cross-Zero Reset**: The integral accumulator automatically drops to ``0`` when crossing the target threshold to eliminate overshoot oscillations.