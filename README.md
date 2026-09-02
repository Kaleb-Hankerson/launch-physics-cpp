# launch-physics-cpp

A C++ port of the physics engine from [launch-to-orbit-sim](https://github.com/Kaleb-Hankerson/launch-to-orbit-sim). It is a 2D rocket ascent-to-orbit simulator. This repo implements the same forces, RK4 integrator, and staged-flight logic in C++.

## Scope

This is the physics engine only, so no visualization, no telemetry pipeline beyond a CSV output. The goal is to demonstrate implementing the same numerical/physics algorithm correctly in a systems language, not to replicate the full feature set of the Python version.

## Physics modeled

- Thrust, atmospheric drag, and gravity, combined into a 2D force model
- A two-stage rocket (thrust, exhaust velocity, and mass flow rate vary per stage; each stage's structural mass is jettisoned on burnout)
- A linear pitch program for ascent steering
- Exponential atmospheric density model
- RK4 (4th-order Runge-Kutta) integration

## Design

- **Structs + free functions, not a class.** `calc_derivatives()` needs to stay a pure function — callable safely on hypothetical RK4 sample states without mutating any persistent object state. Free functions operating on plain structs (`RocketState`, `Stage`, `RocketConfig`) enforce that naturally. This also mirrors how NASA's F´ flight-software framework and general embedded/flight-sim C++ code are commonly structured.
- **Embedded-style discipline:** no dynamic memory allocation after startup (`std::array`, not `std::vector`, for the fixed two-stage configuration), no exceptions, `double` precision throughout (not `float`, to avoid orbital drift over a long simulation).
- **`RocketConfig`** bundles the rocket's fixed, never-changing configuration (stages, initial mass, dry mass, drag coefficient, cross-sectional area) into a single struct, keeping `calc_derivatives()` to 5 parameters.
- **Operator overloading** (`+`, `-`, `*`, `/`) on a `Vector2` struct, so the RK4 stepper's vector arithmetic reads close to the Python version's numpy-array equivalents.

## Validation

The RK4 integrator was validated in isolation against the analytical projectile-motion formula (zero thrust, zero drag, known initial velocity) and it came within ~4×10⁻¹³ m over a 5-second simulation, essentially floating-point precision. This mirrors the validation approach used in the Python version's unit test, and confirms the core integration and gravity mechanics are correct independent of the more complex staged-flight physics.

The full staged-rocket run is in the same order of magnitude as the Python version's equivalent run (apogee, flight duration), with some numerical divergence expected from floating-point accumulation differences between languages/compilers over 15,000+ RK4 steps on a highly eccentric, numerically-sensitive trajectory.

## Known limitations (deliberate simplifications)

- Constant thrust, exhaust velocity, and mass flow rate per stage(no throttle profile)
- Constant drag coefficient that doesn't vary with Mach number (no transonic spike modeled)
- 3DOF (point-mass) model (No rocket orientation or rotational dynamics)
- A fixed linear pitch program, not closed-loop guidance. The rocket does not reliably reach a genuinely stable orbit as a result. Explored alternatives (adding a second stage, a velocity-following gravity turn, retuning the pitch program) all failed to meaningfully improve this. See the companion Python repo's README for the full investigation. Reliable orbital insertion would require real closed-loop guidance (e.g. Linear Tangent Guidance), which is out of scope for this project.

## Building

Requires CMake and a C++17-compatible compiler.

```
git clone https://github.com/Kaleb-Hankerson/launch-physics-cpp
cd launch-physics-cpp
cmake -B build
cmake --build build
./build/launch-physics-cpp
```

Output is written to `telemetry.csv` in the working directory.

## Companion project

See [launch-to-orbit-sim](https://github.com/Kaleb-Hankerson/launch-to-orbit-sim) for the full Python simulation with additional physics (dynamic pressure/Max Q, orbital elements), a flight-phase state machine, unit tests, and CSV telemetry with a standalone analysis script.