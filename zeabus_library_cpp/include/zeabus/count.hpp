// FILE			: count.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 20 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README
//  This file have purpose to count same time and return data for you

// REFERENCE

// MACRO SET

#include    <iostream>

#include    <zeabus/escape_code.hpp>

#ifndef _ZEABUS_COUNT_HPP__
#define _ZEABUS_COUNT_HPP__

namespace zeabus
{

namespace count
{


    // what mean of compare? That mean I will compare with our data
    // return value will return for you about that same data or not
    //      but count_over will return true if same_time > limit_count
    template< typename data_type >
    bool compare( data_type data , unsigned int limit_count , bool* count_over );

    // This use for check in case same data that are over or not
    //  return value will return data for equal that meat we will return equal to you
    //  please send result to us
    bool check( bool equal , unsigned int limit_count , bool* count_over );

} // namespace count

} // namespace zeabus

#include    <zeabus/count.cpp> // this is source code for dynamic part

#endif // _ZEABUS_COUNT_HPP__
