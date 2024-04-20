/********************************* TRICK HEADER *******************************
PURPOSE: ( Simulate a marsr. )
LIBRARY DEPENDENCY:
    ((Marsr.o))
*******************************************************************************/
#include "../include/Marsr.hh"
#include "sim_services/Integrator/include/integrator_c_intf.h"
#include <math.h>
#include <iostream>

int Marsr::default_data() {
    // Initialize the environment using the current altitude
    env.default_data(position[2]);
    motor.default_data();

    // Initialize other Marsr properties
    mission_time = 0.0;
    mission_time_rate = 1.0;
    position[0] = 0.0;
    position[1] = 0.0;
    position[2] = 0.0; // Default altitude, should be set based on actual data
    velocity[0] = 0.0;
    velocity[1] = 0.0;
    velocity[2] = 0.0;
    phi = 0.0;
    omega = 0.0;
    mass_dry = 29620; // Reflects MarsR
    mass_fuel = 228880; // Reflects MarsR
    exhaust_vel = 3087.322; // Reflects MarsR
    mass = mass_dry + mass_fuel;
    return 0;
}

int Marsr::state_init() {
    // Initialize state-specific settings, potentially including additional motor setup
    motor.state_init();
    return 0;
}

int Marsr::state_deriv() {
    // Physics calculations
    thrust_mag = motor.thrust_magnitude(mission_time);
    steering_mag = motor.steer_magnitude(mission_time);
    thrust_force = thrust_mag * env.maxThrust;

    if (mass_fuel > 0) {
        mass_rate = -thrust_force / exhaust_vel;
        if (mass_fuel + mass_rate * mission_time_rate < 0) {
            mass_rate = -mass_fuel / mission_time_rate;  // Adjust mass rate to consume remaining fuel
        }
        mass_fuel += mass_rate * mission_time_rate; // Update fuel mass
    } else {
        mass_rate = 0;
        thrust_force = 0;
    }

    // Update total mass
    mass = mass_dry + mass_fuel;

    // Compute acceleration
    acc[0] = env.gravity[0] + thrust_force * cos(phi) / mass;
    acc[1] = 0.0;
    acc[2] = env.gravity[2] + thrust_force * sin(phi) / mass;
    omega_dot = -steering_mag * env.maxSteer / env.inertia;
    
    // Debugging output
    std::cout << "Mass: " << mass << " kg, Mass Rate: " << mass_rate << " kg/s" << std::endl;

    return 0;
}


int Marsr::state_integ() {
    // Perform state integration
    int integration_step;
    load_state(&mission_time, &position[0], &position[1], &position[2],
               &velocity[0], &velocity[1], &velocity[2],
               &phi, &omega, &mass_fuel, (double *)0);
    load_deriv(&mission_time_rate, &velocity[0], &velocity[1], &velocity[2],
               &acc[0], &acc[1], &acc[2], &omega_dot, &mass_rate, (double *)0);

    integration_step = integrate();
    unload_state(&mission_time, &position[0], &position[1], &position[2],
                 &velocity[0], &velocity[1], &velocity[2],
                 &phi, &omega, &mass_fuel, (double *)0);

    // Update to mass post integration to reflect changes in mass_fuel
    mass = mass_dry + mass_fuel;

    // Debugging output
    std::cout << "Post-Integration Mass: " << mass << " kg" << std::endl;

    return integration_step;
}


