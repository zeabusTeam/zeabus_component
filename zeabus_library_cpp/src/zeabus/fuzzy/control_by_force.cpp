#include <zeabus/fuzzy/control_by_force.hpp>

// double diff(double target, double current)
// {
//     double diff_force = 0;
//     diff_force = target - current;
//     return diff_force;
// }

int out_range = 0;

double choose_range(double error)
{
    if(fabs(error) >= input_force[4])
    {
        out_range = 5;
    }
    else if(fabs(error) >= input_force[3])
    {
        out_range = 4;
    }
    else if(fabs(error) >= input_force[2])
    {
        out_range = 3;
    }
    else if(fabs(error) >= input_force[1])
    {
        out_range = 2;
    }
    else if(fabs(error) >= input_force[0])
    {
        out_range = 1;
    }
    else if(fabs(error) < input_force[0])
    {
        out_range = 0;
    }
    return out_range;
}

double out(int out_range, double error)
{
    double out_add = 0;
    out_add = std::copysign(output_force[out_range], error);
    return out_add;
}

double add(double output_force, double forces)
{
    std::cout << "force " << force << std::endl;
    std::cout << "output_force " << output_force << std::endl;
    force = forces + output_force;
    std::cout << "force " << force << std::endl;
    return force;
}

double run_system(double error)
{
    double output = 0;
    double out_of_choose = 0;
    double out_of_out = 0;
    out_of_choose = choose_range(error);
    std::cout << "out_of_choose " << out_of_choose << std::endl;
    out_of_out = out(out_of_choose, error);
    output = add(out_of_out, force);
    std::cout << "out_of_out " << out_of_out << std::endl;
    std::cout << "output " << output << std::endl;
    return output;
}


