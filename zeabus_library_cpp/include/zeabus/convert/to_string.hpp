// FILE			: to_string.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 10 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README
//  You can read example file at cpp_text package on file cpp_convert.cpp
//  This use template for convert any type to string by std::basic_stringstream< char >

// REFERENCE
//  ref1    : https://en.cppreference.com/w/cpp/io/basic_stringstream
//  ref2    : https://en.cppreference.com/w/cpp/string/char_traits

#include    <sstream>

#ifndef _ZEABUS_CONVERT_TO_STRING_HPP__
#define _ZEABUS_CONVERT_TO_STRING_HPP__

namespace zeabus
{

namespace convert
{

    template< typename input_type >
    std::string to_string( input_type input_data );

} // namespace convert

} // namespace zeabus

#include    <zeabus/convert/to_string.cpp>

#endif // _ZEABUS_CONVERT_TO_STRING_HPP__
