#include <zeabus/fuzzy/control_velocity.hpp>

double pre_error = 0;

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

double find_acceleration(double error)
{
    int acceleration_value = 0;
    acceleration_value = pre_error - error;
    return acceleration_value;
}

int acceleration_range(double acceleration_value)
{
    int acceleration_range_value = 0;
    if(acceleration_value >= -10)
    {
        acceleration_range_value = 0;
    }
    else if(acceleration_value >= -5)
    {
        acceleration_range_value = 1;
    }
    else if(acceleration_value >= 0)
    {
        acceleration_range_value = 2;
    }
    else if(acceleration_value >= 5)
    {
        acceleration_range_value = 3;
    }
    else if(acceleration_value >= 10)
    {
        acceleration_range_value = 4;
    }
    return acceleration_range_value;
}

int choose_fuzzy(int error_range_value, int acceleration_range_value)
{
    int choose_fuzzy_value = 0;
    choose_fuzzy_value = fuzzy[acceleration_range_value][error_range_value];
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
    add_force_value = force[axis] + fuzzy_range_value;
    return add_force_value;
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

int check(double output, int type)
{
    if(type == 0)
    {
        if(output >= 10)
        {
            output = 10;
        }
    }
    else if(type == 1)
    {
        if(output >= 8)
        {
            output = 8;
        }
    }
    else if(type == 2)
    {
        if(output >= 3)
        {
            output = 3;
        }
    }
    return output;
}

double run_system(double error, int axis, bool mask)
{
    int type = 0;
    int error_range_value_out = 0;
    double acceleration_value_out = 0;
    int acceleration_range_value_out = 0;
    int choose_fuzzy_value_out = 0;
    double fuzzy_range_value_out = 0;
    double add_force_value_out = 0;
    if(mask == false)
    {
        force[axis] = 0;
        return force[axis];
    }
    std::cout << "error: " << error << std::endl;
    type = choose_type(axis);
    std::cout << "type: " << type << std::endl;
    error_range_value_out = error_range(error, type);
    std::cout << "error_range_value_out: " << error_range_value_out << std::endl;
    acceleration_value_out = find_acceleration(error);
    std::cout << "acceleration_value_out: " << acceleration_value_out << std::endl;
    acceleration_range_value_out = acceleration_range(acceleration_value_out);
    std::cout << "acceleration_range_value_out: " << acceleration_range_value_out << std::endl;
    choose_fuzzy_value_out = choose_fuzzy(error_range_value_out, acceleration_range_value_out);
    std::cout << "choose_fuzzy_value_out: " << choose_fuzzy_value_out << std::endl;
    fuzzy_range_value_out = fuzzy_range(choose_fuzzy_value_out, type, error);
    std::cout << "fuzzy_range_value_out: " << fuzzy_range_value_out << std::endl;
    add_force_value_out = add_force(fuzzy_range_value_out, axis);
    std::cout << "add_force_value_out: " << add_force_value_out << std::endl;
    pre_error = error;
    std::cout << "pre_error: " << pre_error << std::endl;
    add_force_value_out = check(add_force_value_out, type);
    return add_force_value_out;
}
