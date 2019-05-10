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

namespace zeabus
{

namespace convert
{

    template< typedef input_type >
    std::string to_string( input_type input_data );

} // namespace convert

} // namespace zeabus
