/*************************************************************************
PURPOSE: (Environment for Mars Reacher Sim)
LIBRARY DEPENDENCIES:
    ((marsr/src/Environment.o))
**************************************************************************/
class Environment {
public:
    double mass;        /* kg */
    double inertia;     /* kg*m^2 */
    double cd;          /* 1 */
    double maxThrust;   /* N */
    double maxSteer;    /* N*m */
    double steerDamp;   /* N*s/m */
    int default_data();
};
