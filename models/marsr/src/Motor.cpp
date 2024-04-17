#include "../include/Motor.hh"
#include "../include/interpolate.h"
#include <iostream>

#define NUM_ELEMENTS 11
const double sample_time_array[NUM_ELEMENTS] = {
0, 12, 24, 36, 48, 60, 72, 84, 96, 108, 120};

const double thrust_array[NUM_ELEMENTS] = {
  0, 1, 1, 1, 1, 1, 0.5, 0.5, 0.5, 0.4, 0};

const double steer_array[NUM_ELEMENTS] = {
  0, 1, 1, 1, -1, -1, 0.5, 0.5, 0.5, 0.4, 0};


int Motor::default_data () {

    sample_times = sample_time_array;
    thrust_sample_values = thrust_array;
    steer_sample_values = steer_array;
    thrustNsteer_sample_count = NUM_ELEMENTS;
    state_init();
    return 0;
}

int Motor::state_init () {
  return 0;
}

double Motor::thrust_magnitude(double time) {
    // Call interpolator
    return interpolate( time, sample_times, thrust_sample_values, thrustNsteer_sample_count );
}

double Motor::steer_magnitude(double time) {
    // Call interpolator
    return interpolate( time, sample_times, steer_sample_values, thrustNsteer_sample_count );
}

