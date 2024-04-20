/*************************************************************************
PURPOSE: (Simulate a Mars mission)
LIBRARY DEPENDENCIES:
    ((marsr/src/Marsr.o))
**************************************************************************/
#ifndef _marsr_hh_
#define _marsr_hh_

#include "Environment.hh"
#include "Motor.hh"

class Marsr {
public:
    Environment env;
    Motor motor;

    double thrust_force;
    double drag_force[3];      // Although not used in the provided .cpp snippet, assuming it's part of the physical model
    double total_force[3];     // Total forces on the Marsr (not fully implemented in the provided .cpp snippet)
    double thrust_mag;
    double steering_mag;
    double gravity[3];         // Gravity vector to be calculated in state_deriv

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

    int default_data();
    int state_init();
    int state_deriv();
    int state_integ();

};
#endif

