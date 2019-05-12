// FILE			: integer.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 12 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README
//  I create this file because to implement <ctime> <iomanip> and <sstream> to convert 
//      interger(about time) to string by set width of data

// REFERENCE

// MACRO SET

#include    <iomanip>

#include    <sstream>

#ifndef _ZEABUS_CONVERT_INT_HPP__
#define _ZEABUS_CONVERT_INT_HPP__

namespace zeabus
{

namespace convert
{

namespace integer
{

    std::string to_string( int* data , unsigned int width , char delim = '0' );

} // namespace interger

} // namespace convert

} // namespace zeabus

#endif // _ZEABUS_CONVERT_INT_HPP__
