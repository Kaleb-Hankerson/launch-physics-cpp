#include <iostream>
#include "rocket.h"
#include <fstream>


int main() {
    RocketConfig config;
    config.stages = {{
        {3000, 250, 38000, 4000},
        {3500, 60, 5000, 500}
    }};

    double initial_total_mass = 0;
    for (const Stage& stage : config.stages) {
        initial_total_mass += stage.propellant_mass + stage.structural_mass;
    }
    config.initial_total_mass = initial_total_mass;
    config.final_dry_mass = config.stages[1].structural_mass;

    double launch_latitude = 34.7;
    double earth_angular_velocity = 7.292e-5;
    double earth_radius = 6.371e6;
    double initial_vx = earth_angular_velocity * earth_radius * std::cos(launch_latitude * (M_PI / 180.0));

    RocketState state;
    state.position = {0.0, 0.0};
    state.velocity = {initial_vx, 0.0};
    state.mass = initial_total_mass;
    state.time = 0.0;

    double dt = 0.1;
    int max_steps = 500000;
    int step_count = 0;

    std::ofstream csv_file("telemetry.csv");
    csv_file << "Time,X,Y,VelocityX,VelocityY,Mass\n";

    while (true) {
        state = rk4_step(state, dt, config);
        step_count++;

        csv_file << state.time << "," << state.position.x << "," << state.position.y << ","
                  << state.velocity.x << "," << state.velocity.y << "," << state.mass << "\n";

        if (step_count % 10000 == 0) {
            std::cout << "Step " << step_count << ": t=" << state.time
                      << "s, altitude=" << state.position.y << "m\n";
        }

        if (step_count > max_steps || state.position.y <= 0) {
            break;
        }
    }

    csv_file.close();

    std::cout << "Simulation complete. Final altitude: " << state.position.y << "m at t=" << state.time << "s\n";

    return 0;
}