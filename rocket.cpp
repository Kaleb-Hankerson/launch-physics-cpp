#include "rocket.h"

Vector2 operator+(const Vector2& a, const Vector2& b) {
    return {a.x + b.x, a.y + b.y};
}

Vector2 operator-(const Vector2& a, const Vector2& b) {
    return {a.x - b.x, a.y - b.y};
}

Vector2 operator*(const Vector2& v, double scalar) {
    return {v.x * scalar, v.y * scalar};
}

Vector2 operator/(const Vector2& v, double scalar) {
    return {v.x / scalar, v.y / scalar};
}

double vector_magnitude(const Vector2& v) {
    return std::sqrt(v.x * v.x + v.y * v.y);
}

double calc_thrust(double mass, const Stage& stage, double stage_burnout_mass) {
    if (mass > stage_burnout_mass) {
        return stage.exhaust_velocity * stage.mass_flow_rate;
    } else {
        return 0;
    }
}

double calc_mass_rate(double mass, const Stage& stage, double stage_burnout_mass) {
    if (mass > stage_burnout_mass) {
        return -stage.mass_flow_rate;
    }
    else {
        return 0;
    }
}

double calc_density(const Vector2& position) {
    return SEA_LEVEL_DENSITY * std::exp(-position.y / SCALE_HEIGHT);
}

//Falls through to stages[1] (the final stage) once mass drops below every computed burnout threshold.
//Hardcoded to a 2-stage array and would need generalizing if a
//variable stage count were ever supported.
StageResult get_stage_and_burnout(double mass, const std::array<Stage, 2>& stages, double initial_total_mass, double final_dry_mass) {
    double cumulative_mass = 0;
    for (const Stage& stage : stages) {
        cumulative_mass += stage.propellant_mass + stage.structural_mass;
        double stage_burnout_mass = initial_total_mass - cumulative_mass + stage.structural_mass;
        if (mass > stage_burnout_mass) {
            return {stage, stage_burnout_mass};
        }
    }
    return {stages[1], final_dry_mass};
}

Vector2 calc_drag(const Vector2& velocity, double air_density, double drag_coefficient, double cross_sectional_area) {
    double speed = vector_magnitude(velocity);
    if (speed > 0) {
        double drag_magnitude = 0.5 * air_density * (speed * speed) * drag_coefficient * cross_sectional_area;
        double drag_x = -drag_magnitude * (velocity.x / speed);
        double drag_y = -drag_magnitude * (velocity.y / speed);
        return {drag_x, drag_y};
    } else {
        return {0.0, 0.0};
    }
}

double calc_pitch_angle(double time) {
    if (time < PITCH_DURATION) {
        return PITCH_START_ANGLE - (PITCH_START_ANGLE - PITCH_END_ANGLE) * (time / PITCH_DURATION);
    } else {
        return PITCH_END_ANGLE;
    }
}

Vector2 calc_net_force(double thrust, const Vector2 &drag, double angle, double mass) {
    double angle_radians = angle * (M_PI / 180.0);
    double thrust_x = thrust * std::cos(angle_radians);
    double thrust_y = thrust * std::sin(angle_radians);
    double net_force_x = thrust_x + drag.x;
    double net_force_y = thrust_y - (mass * GRAVITY) + drag.y;
    return {net_force_x, net_force_y};
}


Vector2 calc_accel(const Vector2& net_force, double mass) {
    return net_force / mass;
}

Derivatives calc_derivatives(const Vector2& position, const Vector2& velocity, double mass, double time, const RocketConfig& config) {
    StageResult sr = get_stage_and_burnout(mass, config.stages, config.initial_total_mass, config.final_dry_mass);
    double angle = calc_pitch_angle(time);
    double mass_rate = calc_mass_rate(mass, sr.stage, sr.burnout_mass);
    double thrust = calc_thrust(mass, sr.stage, sr.burnout_mass);
    double air_density = calc_density(position);
    Vector2 drag = calc_drag(velocity, air_density, config.drag_coefficient, config.cross_sectional_area);
    Vector2 net_force = calc_net_force(thrust, drag, angle, mass);
    Vector2 accel = calc_accel(net_force, mass);
    return {velocity, accel, mass_rate};
}

RocketState rk4_step(const RocketState& current, double dt, const RocketConfig& config) {
    Derivatives k1 = calc_derivatives(current.position, current.velocity, current.mass, current.time, config);

    double half_dt = dt / 2;
    Vector2 mid_position_1 = current.position + k1.velocity * half_dt;
    Vector2 mid_velocity_1 = current.velocity + k1.accel * half_dt;
    double mid_mass_1 = current.mass + k1.mass_rate * half_dt;
    double mid_time_1 = current.time + half_dt;

    Derivatives k2 = calc_derivatives(mid_position_1, mid_velocity_1, mid_mass_1, mid_time_1, config);
    Vector2 mid_position_2 = current.position + k2.velocity * half_dt;
    Vector2 mid_velocity_2 = current.velocity + k2.accel * half_dt;
    double mid_mass_2 = current.mass + k2.mass_rate * half_dt;
    double mid_time_2 = current.time + half_dt;

    Derivatives k3 = calc_derivatives(mid_position_2, mid_velocity_2, mid_mass_2, mid_time_2, config);
    Vector2 end_position = current.position + k3.velocity * dt;
    Vector2 end_velocity = current.velocity + k3.accel * dt;
    double end_mass = current.mass + k3.mass_rate * dt;
    double end_time = current.time + dt;

    Derivatives k4 = calc_derivatives(end_position, end_velocity, end_mass, end_time, config);

    Vector2 final_velocity_rate = (k1.velocity + k2.velocity * 2 + k3.velocity * 2 + k4.velocity) / 6;
    Vector2 final_accel = (k1.accel + k2.accel * 2 + k3.accel * 2 + k4.accel) / 6;
    double final_mass_rate = (k1.mass_rate + 2 * k2.mass_rate + 2 * k3.mass_rate + k4.mass_rate) / 6;

    RocketState next;
    next.position = current.position + final_velocity_rate * dt;
    next.velocity = current.velocity + final_accel * dt;
    next.mass = current.mass + final_mass_rate * dt;
    if (next.mass < config.final_dry_mass) {
        next.mass = config.final_dry_mass;
    }
    next.time = current.time + dt;

    return next;
}