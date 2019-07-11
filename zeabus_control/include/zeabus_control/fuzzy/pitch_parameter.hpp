// FILE			: pitch_parameter.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, June 22 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#ifndef _ZEABUS_CONTROL_FUZZY_PITCH_PARAMETER_HPP
#define _ZEABUS_CONTROL_FUZZY_PITCH_PARAMETER_HPP

namespace zeabus_control
{

namespace fuzzy
{

namespace pitch_parameter
{
    // Error is range of error to decision 0 1 2 3 by use 3 value. 
    // Data is vector type
    const std::array< double , 3 > ERROR_RULE = { 0.1 , 1 , 2 };
    
    // Diff is range to decision about velocity error will be 0 1 2 3 by use 3 value
    // Data is vertor type
    const std::array< double , 3 > DIFF_RULE = { 0.05 , 0.20 , 0.40 };
    
    // Force is range to decision now you have active force what range
    // member 3 (indent 0) use to decision that is limit of force we can do
    //  Output will decision to 0 1 2 3 4 by use 4 value
    //  If over member 3 (indent 0) we will not addition force
    //  Data is vector type
    const std::array< double , 4 > FORCE_RULE = { 0.1 , 1 , 2 , 4 };
    
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
    const std::array< double , 5 > DEFUZZY_RULE = {0.01 , 0.02 , 0.04, 0.06, 0.1};
    
    // Offset is value to do and don't have affect with robot in real time always
    const double OFFSET = 0.0;
    
} // namespace pitch_parameter

} // namespace fuzzy 

} // namespace zeabus_control

#endif // _ZEABUS_CONTROL_ROLL_PARAMETER_HPP
