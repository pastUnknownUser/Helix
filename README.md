# Helix - Configurable PID Controller for PROS

A simple, configurable PID controller library for VEX V5 robots built on the PROS framework.

## Features

- **Simple API** - Easy to use with inline documentation and examples
- **Configurable PID** - Tune kP, kI, kD, output limits, and anti-windup
- **Pre-tuned configs** - Start with `PRECISE`, `BALANCED`, or `FAST` presets
- **Built-in settle detection** - Movements complete when robot reaches target
- **IMU support** - Optional heading-based turns using inertial sensor
- **Tank & Arcade drive** - Built-in driver control modes

## Quick Start

### 1. Configure Motors

```cpp
// In main.cpp
pros::Motor leftFront(1, pros::E_MOTOR_GEAR_BLUE, true);
pros::Motor leftBack(2, pros::E_MOTOR_GEAR_BLUE, true);
pros::Motor rightFront(3, pros::E_MOTOR_GEAR_BLUE, false);
pros::Motor rightBack(4, pros::E_MOTOR_GEAR_BLUE, false);

pros::Motor_Group leftSide({leftFront, leftBack});
pros::Motor_Group rightSide({rightFront, rightBack});
pros::IMU imu(10);  // Optional - for accurate turning
```

### 2. Create Drivetrain

```cpp
Helix::Drivetrain drivetrain(
    &leftSide, &rightSide,  // Motor groups
    600,                    // RPM (blue=600, green=200, red=100)
    3.25                    // Wheel diameter in inches
);
```

### 3. Configure Chassis

```cpp
Helix::Chassis::Config config;
config.drivetrain = drivetrain;
config.lateralPID = Helix::PIDController(0.8, 0.001, 0.1);
config.turnPID = Helix::PIDController(1.5, 0.002, 0.15);
config.imu = &imu;  // Optional
config.maxLateralSpeed = 100;
config.maxTurnSpeed = 80;

Helix::Chassis chassis(config);
```

### 4. Autonomous Movements

```cpp
void autonomous() {
    chassis.drive(24);       // Drive forward 24 inches
    chassis.turn(90);        // Turn 90 degrees right
    chassis.drive(-12);      // Drive backward 12 inches
    chassis.turnTo(180);     // Turn to heading 180 degrees (requires IMU)
}
```

## PID Tuning Guide

The PID controller has three gains that control robot behavior:

- **kP (Proportional)** - Responds to current error. Higher = faster response but may oscillate.
- **kI (Integral)** - Eliminates steady-state error. Use sparingly to avoid windup.
- **kD (Derivative)** - Dampens oscillations. Usually set to kP/10 as starting point.

### Tuning Steps

1. **Start with kP only** - Increase until robot oscillates around target
2. **Add kD** - Set to kP/10, adjust to reduce oscillation
3. **Add kI if needed** - Only if robot stops short of target (use very small values)

### Pre-configured Settings

```cpp
// Conservative - accurate but slower
Helix::PIDController precise = Helix::PIDConfigs::PRECISE;

// Balanced - good starting point
Helix::PIDController balanced = Helix::PIDConfigs::BALANCED;

// Aggressive - fast but may overshoot
Helix::PIDController fast = Helix::PIDConfigs::FAST;
```

## Advanced Usage

### Custom Movement Parameters

```cpp
// Drive 48 inches at 80% speed with 5 second timeout
bool success = chassis.drive(48, 80, 5000);
if (!success) {
    // Movement timed out
}
```

### Runtime PID Tuning

```cpp
// Adjust PID gains during testing
chassis.getLateralPID().kP = 1.2;
chassis.getLateralPID().setOutputLimits(-100, 100);
chassis.getLateralPID().setTolerance(0.5, 5);  // 0.5" tolerance, 5 samples
```

### Direct Motor Control

```cpp
// Tank drive
chassis.tank(leftJoystick, rightJoystick);

// Arcade drive
chassis.arcade(forwardInput, turnInput);

// Stop with brake mode
chassis.stop(pros::E_MOTOR_BRAKE_HOLD);
```

## API Reference

### PIDController

| Method | Description |
|--------|-------------|
| `compute(setpoint, measurement)` | Calculate PID output |
| `reset()` | Clear integral and previous error |
| `setOutputLimits(min, max)` | Clamp output range |
| `setIntegralLimit(limit)` | Anti-windup protection |
| `setTolerance(tol, samples)` | Set settle tolerance |
| `isSettled()` | Check if within tolerance |

### Chassis

| Method | Description |
|--------|-------------|
| `drive(dist, speed, timeout)` | Drive distance in inches |
| `turn(angle, speed, timeout)` | Turn relative angle |
| `turnTo(heading, speed, timeout)` | Turn to absolute heading |
| `tank(left, right)` | Direct tank control |
| `arcade(fwd, turn)` | Arcade drive control |
| `stop(mode)` | Stop with brake mode |

## File Structure

```
include/
└── Helix/
    ├── Helix.hpp          # Main include file
    ├── PIDController.hpp  # PID controller class
    └── Chassis.hpp        # Chassis controller class

src/
├── main.cpp              # Example with motor setup
└── Helix/
    ├── PIDController.cpp  # PID implementation
    └── Chassis.cpp        # Chassis implementation
```

## Building

Use PROS CLI commands:

```bash
# Build project
pros build

# Upload to V5 brain
pros upload

# Open terminal to view output
pros terminal
```

Or use Make:

```bash
make quick      # Build only changed files
make all        # Clean rebuild
```

## License

This project is provided as-is for educational use in VEX Robotics competitions.

## Contributing

DM "whoisunknownuser" on Discord to discuss contributions.
