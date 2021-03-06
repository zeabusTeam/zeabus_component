// FILE			: yaw_parameter.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, June 22 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#ifndef _ZEABUS_CONTROL_FUZZY_YAW_PARAMETER_HPP
#define _ZEABUS_CONTROL_FUZZY_YAW_PARAMETER_HPP

namespace zeabus_control
{

namespace fuzzy
{

namespace yaw_parameter
{
    // Error is range of error to decision 0 1 2 3 by use 3 value. 
    // Data is vector type
    const std::array< double , 3 > ERROR_RULE = { 0.1 , 0.2 , 0.6 };
    
    // Diff is range to decision about velocity error will be 0 1 2 3 by use 3 value
    // Data is vertor type
    const std::array< double , 3 > DIFF_RULE = { 0.005 , 0.018 , 0.05 };
    
    // Force is range to decision now you have active force what range
    // member 3 (indent 0) use to decision that is limit of force we can do
    //  Output will decision to 0 1 2 3 4 by use 4 value
    //  If over member 3 (indent 0) we will not addition force
    //  Data is vector type
    const std::array< double , 4 > FORCE_RULE = { 0.02 , 0.1 , 0.5 , 1};
    //  else if( abs(input) < rule[2] ) output = 2
    //  else output = 3 

    // Defuzzy is value ralative to convert from crisp set to fuzzy data
    // Data will have 0, 1, 2, 3, 4.
    // If output is 0 will add 0
    // If output is 1 2 will add by DEFUZZY_RULE member 0 1
    // If output is 3 4 will add by DEFUZZY_RULE member 2
    // Data is verctor type
    const std::array< double , 5 > DEFUZZY_RULE = {0.01 , 0.02 , 0.04, 0.06, 0.12 };
    
    // Offset is value to do and don't have affect with robot in real time always
    const double OFFSET = 0.0;
    
    const std::array< std::array< std::array < int , 7 >, 7 >, 7 > FUZZY_RULE = 
    {
        // CASE fuzzy force = -3
        //  -3  -2  -1  +0  +1  +2  +3  = CASE error fuzzy
            +1, +5, +6, +6, +6, +6, +6, // CASE diff fuzzy = -3
            +0, +5, +4, +6, +5, +6, +6, // CASE diff fuzzy = -2
            -1, +0, +0, +6, +4, +5, +5, // CASE diff fuzzy = -1
            -1, -1, -1, +0, +3, +3, +4, // CASE diff fuzzy = -0
            -4, -4, -4, -6, +0, +0, +3, // CASE diff fuzzy = +1
            -5, -5, -5, -6, -2, -1, +0, // CASE diff fuzzy = +2
            -6, -6, -5, -6, -3, -2, -1, // CASE diff fuzzy = +3
        
        // CASE fuzzy force = -2
        //  -3  -2  -1  +0  +1  +2  +3  = CASE error fuzzy
            +2, +3, +4, +4, +4, +4, +4, // CASE diff fuzzy = -3
            +1, +2, +3, +4, +4, +4, +4, // CASE diff fuzzy = -2
            +0, +0, +0, +3, +3, +3, +4, // CASE diff fuzzy = -1
            -1, -1, -1, +0, +2, +2, +3, // CASE diff fuzzy = -0
            -2, -2, -2, -1, +0, +0, +0, // CASE diff fuzzy = +1
            -3, -2, -2, -1, -1, -1, -1, // CASE diff fuzzy = +2
            -4, -4, -3, -3, -2, -2, -2, // CASE diff fuzzy = +3
        
        // CASE fuzzy force = -1
        //  -3  -2  -1  +0  +1  +2  +3  = CASE error fuzzy
            +1, +2, +3, +4, +4, +4, +5, // CASE diff fuzzy = -3
            +1, +0, +2, +3, +3, +3, +4, // CASE diff fuzzy = -2
            +0, -1, +0, +2, +2, +3, +4, // CASE diff fuzzy = -1
            -2, -1, -1, +0, +2, +2, +2, // CASE diff fuzzy = -0
            -3, -2, -2, -2, +0, +1, +0, // CASE diff fuzzy = +1
            -4, -3, -3, -2, -2, -0, +1, // CASE diff fuzzy = +2
            -5, -4, -4, -4, -3, -2, +1, // CASE diff fuzzy = +3
        
        // CASE fuzzy force = 0
        //  -3  -2  -1  +0  +1  +2  +3  = CASE error fuzzy
            +1, +2, +3, +4, +4, +4, +5, // CASE diff fuzzy = -3
            +0, +2, +3, +3, +3, +3, +4, // CASE diff fuzzy = -2
            -1, +0, +0, +1, +2, +3, +3, // CASE diff fuzzy = -1
            -2, -2, -1, +0, +1, +2, +3, // CASE diff fuzzy = -0
            -3, -3, -2, -1, +0, +0, +1, // CASE diff fuzzy = +1
            -4, -3, -3, -3, -3, -2, +0, // CASE diff fuzzy = +2
            -5, -4, -4, -4, -3, -2, -1, // CASE diff fuzzy = +3
        
        // CASE fuzzy force = +1
        //  -3  -2  -1  +0  +1  +2  +3  = CASE error fuzzy
            +1, +2, +3, +4, +4, +4, +5, // CASE diff fuzzy = -3
            +1, +0, +2, +3, +3, +3, +4, // CASE diff fuzzy = -2
            +0, -1, +0, +1, +2, +2, +3, // CASE diff fuzzy = -1
            -2, -2, -2, +0, +1, +1, +1, // CASE diff fuzzy = -0
            -4, -3, -2, -2, +0, +1, +0, // CASE diff fuzzy = +1
            -4, -3, -3, -3, -2, -0, +1, // CASE diff fuzzy = +2
            -5, -4, -4, -4, -3, -2, -1, // CASE diff fuzzy = +3
        
        // CASE fuzzy force = +2
        //  -3  -2  -1  +0  +1  +2  +3  = CASE error fuzzy
            +2, +2, +2, +3, +3, +4, +4, // CASE diff fuzzy = -3
            +1, +1, +1, +1, +2, +2, +3, // CASE diff fuzzy = -2
            +0, +0, +0, +1, +2, +2, +2, // CASE diff fuzzy = -1
            -3, -2, -2, +0, +1, +1, +1, // CASE diff fuzzy = -0
            -4, -3, -3, -3, +0, +0, +0, // CASE diff fuzzy = +1
            -4, -4, -4, -4, -2, -2, -1, // CASE diff fuzzy = +2
            -4, -4, -4, -4, -2, -2, -1, // CASE diff fuzzy = +3

        // CASE fuzzy force = +3
        //  -3  -2  -1  +0  +1  +2  +3  = CASE error fuzzy
            +1, +2, +3, +6, +4, +6, +6, // CASE diff fuzzy = -3
            +0, +1, +2, +6, +4, +6, +6, // CASE diff fuzzy = -2
            -3, +0, +0, +6, +4, +5, +1, // CASE diff fuzzy = -1
            -4, -3, -3, +0, +1, +1, +1, // CASE diff fuzzy = -0
            -5, -5, -4, -6, +0, +0, +1, // CASE diff fuzzy = +1
            -6, -6, -5, -6, -4, -5, +0, // CASE diff fuzzy = +2
            -6, -6, -6, -6, -6, -5, -2  // CASE diff fuzzy = +3

    };
} // namespace roll_parameter

} // namespace fuzzy 

} // namespace zeabus_control

#endif // _ZEABUS_CONTROL_ROLL_PARAMETER_HPP
