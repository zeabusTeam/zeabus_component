#include <zeabus/fuzzy/control_velocity_output.hpp>

double pre_output = 0;

int error_range(double error, int type)
{
    int error_range_value = 0;
    if(fabs(error) >= input_error[type][3])
    {
        error_range_value = 4;
    }
    else if(fabs(error) >= input_error[type][2])
    {
        error_range_value = 3;
    }
    else if(fabs(error) >= input_error[type][1])
    {
        error_range_value = 2;
    }
    else if(fabs(error) >= input_error[type][0])
    {
        error_range_value = 1;
    }
    else if(fabs(error) < input_error[type][0])
    {
        error_range_value = 0;
    }
    return error_range_value;
}

int pre_output_range(double pre_output, int type)
{
    int pre_output_range_value = 0;
    if(pre_output == output_force[type][0])
    {
        pre_output_range_value = 0;
    }
    else if(pre_output == output_force[type][1])
    {
        pre_output_range_value = 1;
    }
    else if(pre_output == output_force[type][2])
    {
        pre_output_range_value = 2;
    }
    else if(pre_output == output_force[type][3])
    {
        pre_output_range_value = 3;
    }
    else if(pre_output == output_force[type][4])
    {
        pre_output_range_value = 4;
    }
    return pre_output_range_value;
}

int choose_fuzzy(int error_range_value, int pre_output_range_value)
{
    int choose_fuzzy_value = 0;
    choose_fuzzy_value = fuzzy[pre_output_range_value][error_range_value];
    return choose_fuzzy_value;
}

double fuzzy_range(int choose_fuzzy_value, int type, double error)
{
    double fuzzy_range_value = 0;
    fuzzy_range_value = std::copysign(output_force[type][choose_fuzzy_value], error);
    return fuzzy_range_value;
}

double add_force(double fuzzy_range_value, int axis)
{
    double add_force_value = 0;
    std::cout << "force[axis]: " << force[axis] << std::endl;
    std::cout << "fuzzy_range_value: " << fuzzy_range_value << std::endl;
    add_force_value = force[axis] + fuzzy_range_value;
    force[axis] = add_force_value;
    std::cout << "force[axis]: " << force[axis] << std::endl;
    return force[axis];
}

double choose_type(int axis)
{
    int type = 0;
    if(axis == 0 || axis == 1)
    {
        type = 0;
    }
    else if(axis == 2)
    {
        type = 1;
    }
    else if(axis == 3 || axis == 4 || axis == 5)
    {
        type = 2;
    }
    return type;
}

double check(double output, int type, int axis)
{
    if(type == 0)
    {
        if(output >= 10 || output <= -10)
        {
            force[axis] = std::copysign(10, output);
        }
    }
    else if(type == 1)
    {
        if(output >= 8 || output <= -8)
        {
            force[axis] = std::copysign(8, output);
        }
    }
    else if(type == 2)
    {
        if(output >= 1 || output <= -1)
        {
            force[axis] = std::copysign(1, output);
        }
    }
    std::cout << "check_output: " << force[axis] << std::endl;
    return force[axis];
}

int fuzzy_use[5][5] = {};

void copyarrary(int type)
{
    for(int i=0;i<5;i++)
    {
        for(int j=0;j<5;j++)
        {
            fuzzy_use[i][j] = fuzzy[i][j];
        }
    }
    return;
}

void copyarrary_yaw(int type)
{
    for(int i=0;i<5;i++)
    {
        for(int j=0;j<5;j++)
        {
            fuzzy_use[i][j] = fuzzy_yaw[i][j];
        }
    }
    return;
}

double run_system(double error, int axis, bool mask)
{
    int type = 0;
    int error_range_value_out = 0;
    int pre_output_range_value = 0;
    int choose_fuzzy_value_out = 0;
    double fuzzy_range_value_out = 0;
    double add_force_value_out = 0;
    if(mask == false)
    {
        force[axis] = 0;
        return force[axis];
    }
    std::cout << "error: " << error << std::endl;
    std::cout << std::endl;
    type = choose_type(axis);
    std::cout << "type: " << type << std::endl;
    //if(type == 0)
    //{
    //    copyarray(type);
    //}
    //else if(type == 2)
    //{
    //    copyarray_yaw(type);
    //}
    error_range_value_out = error_range(error, type);
    std::cout << "error_range_value_out: " << error_range_value_out << std::endl;
    pre_output_range_value = pre_output_range(pre_output, type);
    std::cout << "pre_output_range_value: " << pre_output_range_value << std::endl;
    choose_fuzzy_value_out = choose_fuzzy(error_range_value_out, pre_output_range_value);
    std::cout << "choose_fuzzy_value_out: " << choose_fuzzy_value_out << std::endl;
    fuzzy_range_value_out = fuzzy_range(choose_fuzzy_value_out, type, error);
    pre_output = fuzzy_range_value_out;
    std::cout << "fuzzy_range_value_out: " << fuzzy_range_value_out << std::endl;
    add_force_value_out = add_force(fuzzy_range_value_out, axis);
    std::cout << "add_force_value_out: " << add_force_value_out << std::endl;
    add_force_value_out = check(add_force_value_out, type, axis);
    std::cout << std::endl;
    return add_force_value_out;
}
