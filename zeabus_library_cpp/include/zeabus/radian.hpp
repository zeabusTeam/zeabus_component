// FILE			: radian.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 18 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README
//  This file will use for assign about double oprator for radian in zabus team
//  This assign we will restrict domain in range [-3.1416 , 3.1416]
//  About algorithm for find shortest part in angle we will use pattern of aliasing for find

// REFERENCE

// MACRO SET

#ifndef _ZEABUS_RADIAN_HPP__
#define _ZEABUS_RADIAN_HPP__

namespace zeabus
{

namespace radian
{

    const static double pi = 3.1416;    // value approximate from MATLAB 
    const static double two_pi = 6.2832;
    const static double negate_pi = -3.1416;
    const static double negate_two_pi = -6.2832;

    // return value after bound
    double bound( double number );

    // same above function but use overload and pointer to do
    void bound( double* number );

    // find vector differenct angle
    double different( double start , double target );
    double different( double* start , double* target);
    void different( double* start , double* target , double* result );

} // namespace radian

} // namespace zeabus

#endif // _ZEABUS_RADIAN_HPP__
