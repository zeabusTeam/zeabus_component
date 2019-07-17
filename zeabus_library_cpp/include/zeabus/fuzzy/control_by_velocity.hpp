#include <iostream>

#include <cmath>

double force[6] = {0, 0, 0, 0, 0, 0};

double input_force[3][5] = {    {0.09, 0.1, 0.3, 0.9, 1.5},
                                {0.09, 0.2, 0.5, 1.0, 1.5},
                                {0.09, 0.15, 0.2, 0.9, 1.5}   };

double output_force[3][6] = {   {0.0, 0.05, 0.1, 0.2, 0.4, 1.0},
                                {0.0, 0.03, 0.06, 0.12, 0.5, 1.0},
                                {0.0, 0.005, 0.05, 0.1, 0.5, 1.5}    };

// double diff(double target, double current);

double choose_range(double error, int type);

double out(int out_range, double error, int type);

double add(double output_force, double forces[6], int axis);

double check_output(double output, int type, unsigned int run);

double run_system(double error, unsigned int run, bool mask);
