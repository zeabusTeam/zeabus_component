#include    <zeabus/fuzzy/control_fuzzy.hpp>


int fuzzy_rule(int diff, int error,int output)
{
    int fuzzy = array_fuzzy[output][diff][error];
    std::cout << "fuzzy : " << fuzzy << std::endl;
    return fuzzy;
}

int check_error(int current, int previous)  // can use return value is array now not done yet
{
    if(current >= 0 && previous >= 0)
    {
        if(current > previous)
        {
            value[0] = current;
            value[1] = previous;
        }
        else
        {
            value[0] = current;
            value[1] = previous;
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
                value[0] = current;
                value[1] = previous;
            }
            else
            {
                value[0] = current;
                value[1] = previous;
            }
        }
    }
    return value[2];
}

int diff(int value[2])
{
    int error;
    error = value[0] - value[1];
    return error;
}

int change_to_fuzzy(int value)
{
    int num=0;
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
    if(crisp < -5)
    {
        error = -3;
    }
    else if(crisp < -2 && crisp >= -5)
    {
        error = -2;
    }
    else if(crisp < 0 && crisp >= -2)
    {
        error = -1;
    }
    else if(crisp == 0)
    {
        error = 0;
    }
    else if(crisp > 0 && crisp <= 2)
    {
        error = 1;
    }
    else if(crisp > 2 && crisp <= 5)
    {
        error = 2;
    }
    else if(crisp > 5)
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
    int num=0;
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
    int num=0;
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
    int num=0;
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
    int num=0;
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

void print(int type, int output, double crisp, int diff)
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
    std::cout << std::endl;
    std::cout << "input error : " << crisp << std::endl;
    std::cout << "diff error : " << diff << std::endl;
    std::cout << "output force : " << output << std::endl;
    std::cout << std::endl;
}

int run_system(double crisp,int type)
{   
    int current_error = 0;
    int value_of_diff;
    int change_error;
    int change_diff;
    int output_fuzzy;
    std::cout << "crisp : " << crisp << std::endl;
    int output;
    if(type == 0 || type == 1 || type == 2)
    {
        current_error = crisp_to_error_xyz(crisp);
    }
    else if(type == 3 || type == 4 || type == 5)
    {
        current_error = crisp_to_error_rpy(crisp);
    }
    std::cout << "previous error : " << previous_error[type] << std::endl;
    std::cout << "current error : " << current_error << std::endl;
    value[2] = check_error(current_error, previous_error[type]);
    previous_error[type] = current_error;
    value_of_diff = diff(value);
    std::cout << value[0] << " " << value[1] << std::endl;
    std::cout << "diff : " << value_of_diff << std::endl;
    change_error = change_to_fuzzy(current_error);
    std::cout << "change_error : " << change_error << std::endl;
    change_diff = change_to_fuzzy(value_of_diff);
    std::cout << "change_diff : " << change_diff << std::endl;
    output_fuzzy = fuzzy_rule(change_diff, change_error, previous_output[type]);
    output_fuzzy = change_to_fuzzy(output_fuzzy);
    std::cout << "output fuzzy : " << output_fuzzy << std::endl;
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
    print(type, output, crisp, value_of_diff);
    return output;
}
