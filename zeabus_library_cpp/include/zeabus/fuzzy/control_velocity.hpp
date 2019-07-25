#include <iostream>

#include <cmath>


double input_error[3][5] = {    {0.09, 0.1, 0.3, 0.9},
                                {0.09, 0.2, 0.5, 1.0},
                                {0.09, 0.15, 0.2, 0.9}   };

double output_force[3][6] = {   {0.0, 0.05, 0.1, 0.2, 0.4},
                                {0.0, 0.03, 0.06, 0.12, 0.5},
                                {0.0, 0.005, 0.05, 0.1, 0.5}    };

double force[6] = {0, 0, 0, 0, 0, 0};

int fuzzy[5][5] = {  {1, 1, 2, 2, 3},
                            {1, 1, 1, 2, 2},
                            {0, 1, 1, 2, 2},
                            {-1, 0, 1, 1, 2},
                            {-1, -1, 0, 1, 1}   };

// double pre_error = 0;

int error_range(double error,int type);

double find_acceleration(double error);

int acceleration_range(double acceleration_value);

int choose_fuzzy(int error_range_value, int acceleration_range_value);

double fuzzy_range(int choose_fuzzy_value, int type, double error);

double add_force(double fuzzy_range_value,int axis);

double choose_type(int axis);

double run_system(double error, int axis, bool mask);