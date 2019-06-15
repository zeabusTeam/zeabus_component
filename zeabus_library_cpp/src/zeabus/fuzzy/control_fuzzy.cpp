#include    <zeabus/fuzzy/control_fuzzy.hpp>

int fuzzy_rule(int diff, int error,int output)
{
    current_output = array_fuzzy[error][diff][output];
}

int check_error(int current, int previous)  // can use return value is array now not done yet
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

int crisp_to_error(double crisp)
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
}

int crisp_to_output(int fuzzy_value)
{
    int num;
    if(fuzzy_value == -3)
    {
        num = 0;
    }
    else if(fuzzy_value == -2)
    {
        num = 1;
    }
    else if(fuzzy_value == -1)
    {
        num = 2;
    }
    else if(fuzzy_value == 0) {
        num = 3;
    }
    else if(fuzzy_value == 1) {
        num = 4;
    }
    else if(fuzzy_value == 2) {
        num = 5;
    }
    else if(fuzzy_value == 3) {
        num = 6;
    }
    return num;
}
