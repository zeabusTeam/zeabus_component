// FILE			: y_parameter.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, June 22 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#ifndef _ZEABUS_CONTROL_FUZZY_Y_PARAMETER_HPP__
#define _ZEABUS_CONTROL_FUZZY_Y_PARAMETER_HPP__

namespace zeabus_control
{

namespace fuzzy
{

namespace y_parameter
{

    // Error is range of error to decision 0 1 2 3 by use 3 value. 
    // Data is vector type
    const std::array< double , 3 > ERROR_RULE = { 0.08 , 0.25 , 3 };
    
    // Diff is range to decision about velocity error will be 0 1 2 3 by use 3 value
    // Data is vertor type
    const std::array< double , 3 > DIFF_RULE = { 0.01 , 0.05 , 0.2 };
    
    // Force is range to decision now you have active force what range
    // member 3 (indent 0) use to decision that is limit of force we can do
    //  Output will decision to 0 1 2 3 4 by use 4 value
    //  If over member 3 (indent 0) we will not addition force
    //  Data is vector type
    const std::array< double , 4 > FORCE_RULE = { 0.05 , 0.19 , 0.39 , 3 };
    
    // About condition to convert crisp_set to fuzzy_set we will use this condition
    //  if( abs( intput ) < rule[0] ) output = 0
    //  else if( abs(input) < rule[1] ) output = 1
    //  else if( abs(input) < rule[2] ) output = 2
    //  else output = 3 

    // Defuzzy is value ralative to convert from crisp set to fuzzy data
    // Data will have 0, 1, 2, 3, 4.
    // If output is 0 will add 0
    // If output is 1 2 will add by DEFUZZY_RULE member 0 1
    // If output is 3 4 will add by DEFUZZY_RULE member 2
    // Data is verctor type
    const std::array< double , 5 > DEFUZZY_RULE = { 0.01 , 0.2, 0.4, 0.8, 1 };
    
    // Offset is value to do and don't have affect with robot in real time always
    const double OFFSET = 0.0;
    
    // fuzzy rule is the main of fuzzyfication for all data
    const std::array< std::array < std::array< int , 7 > , 7 > , 7 > FUZZY_RULE =
        {
        // CASE fuzzy force = -3
        //  -3  -2  -1  +0  +1  +2  +3  = CASE error fuzzy
            +1, +1, +3, +3, +2, +4, +4, // CASE diff fuzzy = -3
            +0, +3, +3, +3, +2, +4, +4, // CASE diff fuzzy = -2
            +0, +0, +0, +3, +1, +1, +1, // CASE diff fuzzy = -1
            -1, -1, -1, +0, +1, +1, +1, // CASE diff fuzzy = -0
            -1, -1, -1, -1, +0, +0, +0, // CASE diff fuzzy = +1
            -2, -2, -2, -1, -1, -1, -1, // CASE diff fuzzy = +2
            -2, -2, -2, -1, -1, -1, -1  // CASE diff fuzzy = +3
        
        , // CASE fuzzy force = -2
        //  -3  -2  -1  +0  +1  +2  +3  = CASE error fuzzy
            +1, +1, +2, +2, +2, +3, +3, // CASE diff fuzzy = -3
            +0, +0, +2, +2, +2, +3, +3, // CASE diff fuzzy = -2
            +0, +0, +0, +2, +1, +1, +1, // CASE diff fuzzy = -1
            -1, -1, -1, +0, +1, +1, +1, // CASE diff fuzzy = -0
            -1, -1, -1, -1, +0, +0, +0, // CASE diff fuzzy = +1
            -2, -2, -2, -1, -1, -1, -1, // CASE diff fuzzy = +2
            -2, -2, -2, -1, -1, -1, -1  // CASE diff fuzzy = +3
        
        , // CASE fuzzy force = -1
        //  -3  -2  -1  +0  +1  +2  +3  = CASE error fuzzy
            +1, +1, +1, +1, +2, +2, +2, // CASE diff fuzzy = -3
            +1, +1, +1, +1, +2, +2, +2, // CASE diff fuzzy = -2
            +0, +0, +0, +1, +1, +1, +1, // CASE diff fuzzy = -1
            -1, -1, -1, +0, +1, +1, +1, // CASE diff fuzzy = -0
            -1, -1, -1, -1, +0, +0, +0, // CASE diff fuzzy = +1
            -2, -2, -2, -1, -1, -1, -1, // CASE diff fuzzy = +2
            -2, -2, -2, -1, -1, -1, -1  // CASE diff fuzzy = +3
        
        , // CASE fuzzy force = 0
        //  -3  -2  -1  +0  +1  +2  +3  = CASE error fuzzy
            +2, +2, +2, +2, +2, +2, +2, // CASE diff fuzzy = -3
            +2, +2, +2, +2, +2, +2, +2, // CASE diff fuzzy = -2
            -0, +0, +0  +1, +2, +2, +2, // CASE diff fuzzy = -1
            -2, -2, -2, +0, +2, +2, +2, // CASE diff fuzzy = -0
            -2, -2, -2, -1, +0, +0, +2, // CASE diff fuzzy = +1
            -2, -2, -2, -2, -2, -2, -2, // CASE diff fuzzy = +2
            -2, -2, -2, -2, -2, -2, -2  // CASE diff fuzzy = +3
        
        , // CASE fuzzy force = +1
        //  -3  -2  -1  +0  +1  +2  +3  = CASE error fuzzy
            +1, +1, +1, +1, +2, +2, +2, // CASE diff fuzzy = -3
            +1, +1, +1, +1, +2, +2, +2, // CASE diff fuzzy = -2
            +0, +0, +0, +1, +1, +1, +1, // CASE diff fuzzy = -1
            -1, -1, -1, +0, +1, +1, +1, // CASE diff fuzzy = -0
            -1, -1, -1, -1, +0, +0, +0, // CASE diff fuzzy = +1
            -2, -2, -2, -1, -1, -1, -1, // CASE diff fuzzy = +2
            -2, -2, -2, -1, -1, -1, -1  // CASE diff fuzzy = +3
        
        , // CASE fuzzy force = +2
        //  -3  -2  -1  +0  +1  +2  +3  = CASE error fuzzy
            +1, +1, +1, +1, +2, +2, +2, // CASE diff fuzzy = -3
            +1, +1, +1, +1, +2, +2, +2, // CASE diff fuzzy = -2
            +0, +0, +0, +1, +1, +1, +1, // CASE diff fuzzy = -1
            -1, -1, -1, +0, +1, +1, +1, // CASE diff fuzzy = -0
            -1, -1, -1, -2, +0, +0, +0, // CASE diff fuzzy = +1
            -3, -3, -2, -2, -2, +0, +0, // CASE diff fuzzy = +2
            -3, -3, -2, -2, -2, -1, -1  // CASE diff fuzzy = +3
        
        , // CASE fuzzy force = +3
        //  -3  -2  -1  +0  +1  +2  +3  = CASE error fuzzy
            +1, +1, +1, +1, +2, +2, +2, // CASE diff fuzzy = -3
            +1, +1, +1, +1, +2, +2, +2, // CASE diff fuzzy = -2
            +0, +0, +0, +1, +1, +1, +1, // CASE diff fuzzy = -1
            -1, -1, -1, +0, +1, +1, +1, // CASE diff fuzzy = -0
            -1, -1, -1, -3, +0, +0, +0, // CASE diff fuzzy = +1
            -4, -4, -2, -3, -3, -3, -0, // CASE diff fuzzy = +2
            -4, -4, -2, -3, -3, -1, -1  // CASE diff fuzzy = +3
        
    };
} // namespace y_parameter

} // namespace fuzzy

} // namespace zeabus_control

#endif
