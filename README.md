# Helix

Helix is a PROS 4 framework for VEX V5 robots. It provides reusable drivetrain
control, PID utilities, and a touchscreen autonomous routine selector.

## Features

- `Helix::Chassis` movement and turning control for paired `pros::MotorGroup`s
- Hardware-agnostic `Helix::PID` control with output, integral, and tolerance limits
- LVGL-based autonomous routine selector for the V5 Brain
- Sphinx documentation for the public API in [`docs/`](docs/)

## Requirements

- VEX V5 hardware
- [PROS CLI](https://pros.cs.purdue.edu/)
- A PROS V5 project toolchain

## Build and deploy

From the project root:

```sh
pros make
pros upload
```

The compiled binaries are written to `bin/`. To build the reusable library
archive instead, run:

```sh
make library
```

## Project layout

| Path | Purpose |
| --- | --- |
| `include/Helix/` | Public Helix headers |
| `src/helix/` | Helix implementations |
| `src/main.cpp` | PROS competition callbacks and autonomous registration |
| `src/auto.cpp` | Robot hardware configuration and autonomous routines |
| `docs/` | Sphinx API documentation |
| `Makefile` | PROS build and library configuration |

## Getting started

Configure the robot hardware and PID values in [`src/auto.cpp`](src/auto.cpp),
then register routines in [`src/main.cpp`](src/main.cpp):

```cpp
auton_selector.autons_add({
    Helix::Auton("Drive test", moveStraight,
                 "Drive forward 10 inches", true, 0),
});
```

Initialize the selector during `initialize()` and execute the selected routine
from `autonomous()` as shown in the included example. Chassis commands use
inches for `move()` and degrees for `turn()`.

## Documentation

Install the documentation dependencies from [`docs/requirements.txt`](docs/requirements.txt),
then build the HTML documentation from the repository root:

```sh
python -m pip install -r docs/requirements.txt
sphinx-build -b html docs docs/_build/html
```

The generated site is available at `docs/_build/html/index.html`.

## License

No license file is currently included in this repository.
