/*************************************************************************
PURPOSE: (Simulate a Mars mission)
LIBRARY DEPENDENCIES:
    ((marsr/src/Marsr.o))
**************************************************************************/
#ifndef _marsr_hh_
#define _marsr_hh_

#include "Environment.hh"
#include "Motor.hh"
#include <string>

class Marsr {
public:
    Environment env;
    Motor motor;

    double thrust_force;
    double drag_force[3];
    double total_force[3];
    double thrust_mag;
    double steering_mag;

    double mission_time;       /* s    time elapsed since launch */
    double mission_time_rate;
    double position[3];        /* m    xyz-position             */
    double velocity[3];        /* m/s  xyz-velocity             */
    double acc[3];             /* m/s2 xyz-acceleration         */
    double phi;                /* 1    azimuth                  */
    double omega;              /* 1/s  angular rate             */
    double omega_dot;          /* 1/s2 angular acceleration     */
    double mass;               /* kg   mass (total)             */
    double mass_dry;           /* kg   dry mass of the rocket   */
    double mass_fuel;          /* kg   fuel mass                */
    double mass_rate;          /* kg/s fuel rate                */
    double exhaust_vel;        /* m/s  exhaust velocity         */
    std::string debug_log;     // To keep debug logs if needed

    int default_data();
    int state_init();
    int state_deriv();
    int state_integ();
    void log_debug(const std::string& message); // Method to log debugging information

    Marsr();                   // Constructor
    ~Marsr();                  // Destructor
};
#endif