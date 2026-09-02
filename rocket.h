#ifndef LAUNCH_TO_ORBIT_SIM_CPP_ROCKET_H
#define LAUNCH_TO_ORBIT_SIM_CPP_ROCKET_H
#include <array>
#include <cmath>


//Core physics engine for the launch-to-orbit rocket simulation, ported from a Python
//implementation. Uses structs + free functions (not a class) so calc_derivatives can
//stay a pure function safely callable on hypothetical RK4 sample states. Written with
//embedded-style discipline: no dynamic allocation after startup (fixed-size std::array
//for stages), no exceptions, double precision throughout to avoid orbital drift.

constexpr double SEA_LEVEL_DENSITY = 1.225;
constexpr double SCALE_HEIGHT = 8500.0;

constexpr double PITCH_START_ANGLE = 90.0;
constexpr double PITCH_END_ANGLE = 45.0;
constexpr double PITCH_DURATION = 100.0;
constexpr double GRAVITY = 9.8;


struct Vector2 {
    double x;
    double y;
};

struct RocketState {
    Vector2 position;
    Vector2 velocity;
    double mass;
    double time;
};

struct Stage {
    double exhaust_velocity;
    double mass_flow_rate;
    double propellant_mass;
    double structural_mass;
};

struct StageResult {
    Stage stage;
    double burnout_mass;
};

// Bundles the rocket's fixed configuration (never changes during flight) into one
// parameter, keeping calc_derivatives under ~5 arguments
struct RocketConfig {
    std::array<Stage, 2> stages;
    double initial_total_mass;
    double final_dry_mass;
    double drag_coefficient;
    double cross_sectional_area;
};

struct Derivatives {
    Vector2 velocity;
    Vector2 accel;
    double mass_rate;
};

Vector2 operator+(const Vector2& a, const Vector2& b);
Vector2 operator-(const Vector2& a, const Vector2& b);
Vector2 operator*(const Vector2& v, double scalar);
Vector2 operator/(const Vector2& v, double scalar);



double calc_thrust(double mass, const Stage& stage, double stage_burnout_mass);
double calc_mass_rate(double mass, const Stage& stage, double stage_burnout_mass);
StageResult get_stage_and_burnout(double mass, const std::array<Stage, 2>& stages, double initial_total_mass, double final_dry_mass);
double calc_density(const Vector2& position);
double vector_magnitude(const Vector2& v);
Vector2 calc_drag(const Vector2& velocity, double air_density, double drag_coefficient, double cross_sectional_area);
double calc_pitch_angle(double time);
Vector2 calc_net_force(double thrust, const Vector2& drag, double angle, double mass);
Vector2 calc_accel(const Vector2& net_force, double mass);
Derivatives calc_derivatives(const Vector2& position, const Vector2& velocity, double mass, double time, const RocketConfig& config);
RocketState rk4_step(const RocketState& current, double dt, const RocketConfig& config);







#endif //LAUNCH_TO_ORBIT_SIM_CPP_ROCKET_H