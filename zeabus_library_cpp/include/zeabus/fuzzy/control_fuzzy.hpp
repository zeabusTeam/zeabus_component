#include <iostream>

int array_fuzzy[7][7][7] =  {   { {1,1,2,3,2,3,3}, {1,1,2,3,2,2,3}, {1,1,2,3,2,2,3}, {0,1,2,3,3,3,3}, {-1,-1,0,2,1,2,2}, {-2,-1,-1,1,1,1,2}, {-3,-2,-1,0,1,2,3} },
                                { {1,1,2,3,2,3,3}, {1,1,2,3,2,2,3}, {1,1,2,3,2,2,3}, {-1,0,1,2,2,2,3}, {-1,-1,0,1,1,2,2}, {-2,-1,-1,0,1,1,2}, {-3,-2,-1,-1,0,0,1} },
                                { {1,1,2,3,2,3,3}, {0,1,1,3,2,2,3}, {0,0,1,2,2,2,3}, {-2,-1,-1,1,1,2,3}, {-2,-2,-1,0,1,1,2}, {-3,-2,-2,-1,0,1,1}, {-3,-3,-2,-2,-1,0,1} },
                                { {0,0,1,3,2,3,3}, {-1,0,0,2,1,2,2}, {-2,-1,0,1,1,2,3}, {-3,-2,-1,0,1,2,3}, {-3,-2,-2,-1,0,1,2}, {-3,-2,-2,-2,-1,0,0}, {-3,-3,-2,-3,-1,0,0} },
                                { {-1,0,1,2,2,2,3}, {-1,-1,0,1,1,2,2}, {-2,-2,-1,0,1,1,2}, {-2,-2,-1,-1,1,1,2}, {-2,-2,-2,-2,-1,0,1}, {-3,-2,-2,-3,-1,-1,0}, {-3,-3,-3,-3,-1,-1,-1} },
                                { {-1,0,0,1,0,0,1}, {-1,-1,0,0,0,1,1}, {-2,-2,-1,-1,0,1,1}, {-2,-2,-1,-2,-1,0,0}, {-2,-2,-2,-3,-1,-1,0}, {-3,-2,-2,-3,-1,-1,0}, {-3,-3,-3,-3,-1,-1,-1} },
                                { {-3,-2,-1,0,0,0,1}, {-2,-2,-1,-1,0,0,1}, {-2,-2,-1,-2,-1,0,1}, {-3,-2,-1,-3,-1,-1,0}, {-3,-3,-2,-3,-1,-1,0}, {-3,-3,-3,-3,-2,-1,-1}, {-3,-3,-3,-3,-2,-2,-2}}  };

int previous_error[6] = {0,0,0,0,0,0};

int previous_output[6] = {0,0,0,0,0,0};

int value1,value2;

int error_case[5];

int error_case2[5] = {-1,-1,-2,-3,-3};

int error_case3[5] = {1,1,2,3,3};

int fuzzy_rule(int diff, int error, int output);  // this function will get diff_error, current_error and previous_output

void check_error(int current, int previous);    // this function will get previous_error and current_error to check previous_error and current_error in each condition

int diff(int value1, int value2);  // this function will get value from check_error function

int change_to_fuzzy(int value);   // this function will get value from current_error and previous_error to change value before used in fuzzy

int crisp_to_error_xyz(double crisp);   // this function will get value from crisp that be x y z for change before input to current_error

int crisp_to_error_rpy(double crisp);   // this function will get value from crisp that be roll pitch yaw for change before input to current_error

int crisp_to_output_x(int fuzzy_value); // this function will get value from output_fuzzy for change before output value from system

int crisp_to_output_y(int fuzzy_value);

int crisp_to_output_z(int fuzzy_value);

int crisp_to_output_rpy(int fuzzy_value);

int run_system(double crisp ,int type);   // this function will run all function in system fuzzy

void print(int type, int output);