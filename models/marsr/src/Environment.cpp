#include "../include/Environment.hh"
#include <math.h>

int Environment::default_data(double altitude) {
    // Constants
    const double g_0 = -1.62; // Standard gravity on Mars at a reference altitude
    const double r_0 = 1737.0e3; // Moon's mean radius in meters
    
    // Initialize environment properties
    gravity[0] = 0.0;
    gravity[1] = 0.0;
    gravity[2] = g_0 * pow(r_0 / (r_0 + altitude), 2);
    inertia = 200;        /* kg*m^2 */
    cd = 500;             /* 1 */
    maxThrust = 200000;   /* N */
    maxSteer = 50;        /* N*m */
    steerDamp = 500;      /* N*s/m */
    
    return 0;
}


