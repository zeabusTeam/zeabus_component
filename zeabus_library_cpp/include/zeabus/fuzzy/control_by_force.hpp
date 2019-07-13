#include <iostream>

#include <cmath>

double force = 0;

double input_force[5] = {0.1, 0.2, 0.5, 1.0, 1.5};

double output_force[6] = {0.0, 0.05, 0.1, 0.2, 0.5, 1.0};

// double diff(double target, double current);

double choose_range(double error);

double out(double out_range, double error);

double add(double output_force, double forces);

double run_system(double error);
