#include "../include/Environment.hh"
#include <math.h>

int Environment::default_data() {

    inertia = 200;        /* kg*m^2 */
    cd = 500;             /* 1 */
    maxThrust = 845000;   /* N  Updated for MarsR*/
    maxSteer = 50;        /* N*m */
    steerDamp = 500;      /* N*s/m */
    
    return 0;
}


