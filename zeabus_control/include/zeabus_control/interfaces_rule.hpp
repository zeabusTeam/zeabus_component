// FILE			: interfaces_rule.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, July 14 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

namespace zeabus_control
{

namespace interfaces
{

namespace xy
{
    static std::array< double , 3 > error_rule = { 0.1 , 1 , 5 };
    static std::array< double , 3 > target_velocity = { 0.1 , 0.25 , 0.5 };
}

namespace z
{
    static std::array< double , 3 > error_rule = { 0.1 , 0.8 , 2 };
    static std::array< double , 3 > target_velocity = { 0.1 , 0.3 , 0.5};
}

namespace zero
{
    static std::array< double , 3 > error_rule = { 0.0 , 0.0 , 0.0 };
    static std::array< double , 3 > target_velocity = { 0.0, 0.0, 0.0};
}

namespace yaw
{
    static std::array< double , 3 > error_rule = { 0.1 , 0.5 , 1 };
    static std::array< double , 3 > target_velocity = { 0.1 , 0.2 , 0.4 };
}

} // namespace interfaces

} // namespace zeabus_control
