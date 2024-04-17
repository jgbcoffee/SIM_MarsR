/*************************************************************************
PURPOSE: (Simulate a rocket motor and steering.)
LIBRARY DEPENDENCIES:
    ((marsr/src/Motor.o)
    (marsr/src/interpolate.o))
**************************************************************************/
class Motor {
public:
    const double * sample_times;
    const double * thrust_sample_values;
    const double * steer_sample_values;

    unsigned int thrustNsteer_sample_count;

    int default_data ();
    int state_init ();
    double thrust_magnitude(double time);
    double steer_magnitude(double time);
};
