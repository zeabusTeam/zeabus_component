#include    <zeabus/fuzzy/control_fuzzy.hpp>


int fuzzy_rule(int diff, int error,int output)
{
    int fuzzy = array_fuzzy[error][diff][output];
    return fuzzy;
}

void check_error(int current, int previous)  // can use return value is array now not done yet
{
    if(current >= 0 && previous >= 0)
    {
        if(current > previous)
        {
            value1 = current;
            value2 = previous;
        }
        else
        {
            value1 = current;
            value2 = previous;
        }
    }
    else if(current < 0 && previous >= 0)
    {
        for(int i=0;i<5;i++) {
            error_case[i] = error_case2[i];
        }
    }
    else if(current >= 0 && previous < 0)
    {
        for(int i=0;i<5;i++) {
            error_case[i] = error_case3[i];
        }
    }
    else if(current < 0 && previous < 0)
    {
        if(current >= 0 && previous >= 0)
        {
            if(current > previous)
            {
                value1 = current;
                value2 = previous;
            }
            else
            {
                value1 = current;
                value2 = previous;
            }
        }
    }
}
int diff(int value1, int value2)
{
    int error;
    error = value1 - value2;
    return error;
}

int change_to_fuzzy(int value)
{
    int num;
    if(value == -3)
    {
        num = 0;
    }
    else if(value == -2)
    {
        num = 1;
    }
    else if(value == -1)
    {
        num = 2;
    }
    else if(value == 0) {
        num = 3;
    }
    else if(value == 1) {
        num = 4;
    }
    else if(value == 2) {
        num = 5;
    }
    else if(value == 3) {
        num = 6;
    }
    return num;
}

int crisp_to_error_xyz(double crisp)
{
    int error = 0;
    if(crisp < -50)
    {
        error = -3;
    }
    else if(crisp < -25 && crisp >= -50)
    {
        error = -2;
    }
    else if(crisp < 0 && crisp >= -25)
    {
        error = -1;
    }
    else if(crisp == 0)
    {
        error = 0;
    }
    else if(crisp > 0 && crisp <= 25)
    {
        error = 1;
    }
    else if(crisp > 25 && crisp <= 50)
    {
        error = 2;
    }
    else if(crisp > 50)
    {
        error = 3;
    }
    return error;
}

int crisp_to_error_rpy(double crisp)
{
    int error = 0;
    if(crisp < -2 && crisp >= -3.14)
    {
        error = -3;
    }
    else if(crisp < -1 && crisp >= -2)
    {
        error = -2;
    }
    else if(crisp < 0 && crisp >= -1)
    {
        error = -1;
    }
    else if(crisp == 0)
    {
        error = 0;
    }
    else if(crisp > 0 && crisp <= 1)
    {
        error = 1;
    }
    else if(crisp > 1 && crisp <= 2)
    {
        error = 2;
    }
    else if(crisp > 2 && crisp <= 3.14)
    {
        error = 3;
    }
    return error;
}

int crisp_to_output_x(int fuzzy_value)
{
    int num;
    if(fuzzy_value == -3)
    {
        num = -6;
    }
    else if(fuzzy_value == -2)
    {
        num = -4;
    }
    else if(fuzzy_value == -1)
    {
        num = -2;
    }
    else if(fuzzy_value == 0) {
        num = 0;
    }
    else if(fuzzy_value == 1) {
        num = 2;
    }
    else if(fuzzy_value == 2) {
        num = 4;
    }
    else if(fuzzy_value == 3) {
        num = 6;
    }
    return num;
}

int crisp_to_output_y(int fuzzy_value)
{
    int num;
    if(fuzzy_value == -3)
    {
        num = -8;
    }
    else if(fuzzy_value == -2)
    {
        num = -5;
    }
    else if(fuzzy_value == -1)
    {
        num = -3;
    }
    else if(fuzzy_value == 0) {
        num = 0;
    }
    else if(fuzzy_value == 1) {
        num = 3;
    }
    else if(fuzzy_value == 2) {
        num = 5;
    }
    else if(fuzzy_value == 3) {
        num = 8;
    }
    return num;
}

int crisp_to_output_z(int fuzzy_value)
{
    int num;
    if(fuzzy_value == -3)
    {
        num = -5;
    }
    else if(fuzzy_value == -2)
    {
        num = -3;
    }
    else if(fuzzy_value == -1)
    {
        num = -1;
    }
    else if(fuzzy_value == 0) {
        num = 0;
    }
    else if(fuzzy_value == 1) {
        num = 1;
    }
    else if(fuzzy_value == 2) {
        num = 3;
    }
    else if(fuzzy_value == 3) {
        num = 5;
    }
    return num;
}

int crisp_to_output_rpy(int fuzzy_value)
{
    int num;
    if(fuzzy_value == -3)
    {
        num = -2;
    }
    else if(fuzzy_value == -2)
    {
        num = -1.5;
    }
    else if(fuzzy_value == -1)
    {
        num = -1;
    }
    else if(fuzzy_value == 0) {
        num = 0;
    }
    else if(fuzzy_value == 1) {
        num = 1;
    }
    else if(fuzzy_value == 2) {
        num = 1.5;
    }
    else if(fuzzy_value == 3) {
        num = 2;
    }
    return num;
}

void print(int type, int output)
{
    // std::string x,y,z,r,p,y;
    // std::string axis[6] = {x,y,z,r,p,yaw};
    // std::cout << "axis : " << axis[type];
    if(type == 0)
    {
        std::cout << "axis : x";
    }
    else if(type == 1)
    {
        std::cout << "axis : y";
    }
    else if(type == 2)
    {
        std::cout << "axis : z";
    }
    else if(type == 3)
    {
        std::cout << "axis : roll";
    }
    else if(type == 4)
    {
        std::cout << "axis : pitch";
    }
    else if(type == 5)
    {
        std::cout << "axis : yaw";
    }
    // std::cout << std::endl;
    std::cout << "  ";
    std::cout << "output force : " << output << std::endl;
}

int run_system(double crisp,int type)
{
    std::cout << "input error : " << crisp << std::endl;
    int current_error;
    int value_of_diff;
    int change_error;
    int change_diff;
    int output_fuzzy;
    int output;
    if(type == 0 || type == 1 || type == 2)
    {
        current_error = crisp_to_error_xyz(crisp);
    }
    else if(type == 3 || type == 4 || type == 5)
    {
        current_error = crisp_to_error_rpy(crisp);
    }
    check_error(current_error, previous_error[type]);
    previous_error[type] = current_error;
    value_of_diff = diff(value1, value2);
    change_error = change_to_fuzzy(current_error);
    change_diff = change_to_fuzzy(value_of_diff);
    output_fuzzy = fuzzy_rule(change_diff, change_error, previous_output[type]);
    previous_output[type] = output_fuzzy;
    if(type == 0)
    {
        output = crisp_to_output_x(output_fuzzy);
    }
    else if(type == 1)
    {
        output = crisp_to_output_y(output_fuzzy);
    }
    else if(type == 2)
    {
        output = crisp_to_output_z(output_fuzzy);
    }
    else if(type == 3 || type == 4 || type == 5)
    {
        output = crisp_to_output_rpy(output_fuzzy);
    }
    print(type, output);
    return output;
}