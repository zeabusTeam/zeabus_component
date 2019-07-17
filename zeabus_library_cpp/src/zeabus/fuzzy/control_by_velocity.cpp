#include <zeabus/fuzzy/control_by_velocity.hpp>

// double diff(double target, double current)
// {
//     double diff_force = 0;
//     diff_force = target - current;
//     return diff_force;
// }

int out_range = 0;

double choose_range(double error, int type)
{
    if(fabs(error) >= input_force[type][4])
    {
        out_range = 5;
    }
    else if(fabs(error) >= input_force[type][3])
    {
        out_range = 4;
    }
    else if(fabs(error) >= input_force[type][2])
    {
        out_range = 3;
    }
    else if(fabs(error) >= input_force[type][1])
    {
        out_range = 2;
    }
    else if(fabs(error) >= input_force[type][0])
    {
        out_range = 1;
    }
    else if(fabs(error) < input_force[type][0])
    {
        out_range = 0;
    }
    return out_range;
}

double out(int out_range, double error, int type)
{
    double out_add = 0;
    out_add = std::copysign(output_force[type][out_range], error);
    return out_add;
}

double add(double output_force, double forces[6], int axis)
{
    std::cout << "force " << force[axis] << std::endl;
    std::cout << "output_force " << output_force << std::endl;
    force[axis] = forces[axis] + output_force;
    std::cout << "force " << force[axis] << std::endl;
    return force[axis];
}

double check_output(double output, int type, unsigned int run)
{
    double out = 0;
    if(type == 0 && output >= 8 && output <= -8)
    {
        out = 8;
    }
    else if(type == 1 && output >= 10 && output <= -10)
    {
        out = 10;
    }
    else if(type == 2 && output >= 3 && output <= -3)
    {
        out = 3;
    }
    out = std::copysign(out, output);
    force[run] = out;
    std::cout << "out" << out << std::endl;
    return out;
}

double run_system(double error, unsigned int run, bool mask)
{   double output = 0;
    double out_of_choose = 0;
    double out_of_out = 0;
    int type = 0;
    if(run == 0 || run == 1)
    {
        type = 0;
    }
    else if(run == 2)
    {
        type = 1;
    }
    else if(run == 3 || run == 4 || run == 5)
    {
        type = 2;
    }
    std::cout << "axis: " << run << std::endl;
    // std::cout << "type: " << type << std::endl;
    if(mask == false)
    {
        force[run] = 0;
        return force[run];
    }
    out_of_choose = choose_range(error, type);
    std::cout << "out_of_choose " << out_of_choose << std::endl;
    out_of_out = out(out_of_choose, error, type);
    output = add(out_of_out, force, run);
    std::cout << "out_of_out " << out_of_out << std::endl;
    std::cout << "output " << output << std::endl;
    std::cout << std::endl;
    if(output >= 3 && output <= -3)
    {
        std::cout << "true" << std::endl;
        output = check_output(output, type, run);
    }
    return output;
}


