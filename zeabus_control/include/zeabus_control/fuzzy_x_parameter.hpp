// FILE			: fuzzy_x_parameter.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, June 13 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README
//  This parameter for fuzzy_3_dimension of control package in linear x

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <array>

#ifndef _ZEABUS_CONTROL_FUZZY_X_PARAMETER_HPP__
#define _ZEABUS_CONTROL_FUZZY_X_PARAMETER_HPP__

namespace zeabus_control
{

namespace fuzzy
{

namespace x_parameter
{
    // Error is range of error to decision 0 1 2 3 by use 3 value. 
    // Data is vector type
    const static std::array< double , 3 > ERROR_RULE = { 0.05 , 0.5 , 2 };

    // Diff is range to decision about velocity error will be 0
    // Data is vertor type
    const static std::array< double , 3 > DIFF_RULE = { 0.02 , 0.13 , 0.25 };

    // Force is range to decision now you have active force what range
    // member 4 (indent 0) use to decision that is limit of force we can do
    //  If over member 4 (indent 0) we will not addition force
    //  Data is vector type
    const static std::array< double , 4 > FORCE_RULE = { 0.1 , 1 , 3 , 6 };

    // Defuzzy is value ralative to convert from crisp set to fuzzy data
    const static std::array< double , 3 > DEFUZZY_RULE = {0.01 , 0.04 , 0.1 };

    // fuzzy rule is the main of fuzzyfication for all data
}

} // namespace fuzzy

} // namespace zeabus_control

#endif
