// FILE			: convert.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 21 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README
//  In this file we help you convert array to string only

// REFERENCE

// MACRO SET

#include    <iostream>

#include    <zeabus/convert/to_string.hpp>

#ifndef _ZEABUS_ARRAY_CONVERT_HPP__
#define _ZEABUS_ARRAY_CONVERT_HPP__

namespace zeabus
{

namespace array
{

namespace convert
{

    std::string to_string( const double* data, const unsigned int size , const char delim);

} // namespace convert

} // namespace array

} // namespace zeabus

#endif
