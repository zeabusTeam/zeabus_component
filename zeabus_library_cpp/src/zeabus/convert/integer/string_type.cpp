// FILE			: string_type.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 12 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

#include    <zeabus/convert/integer.hpp>

namespace zeabus
{

namespace convert
{

namespace integer
{

    std::string to_string( int* data , unsigned int width , char delim )
    {
        std::basic_stringstream< char > temporary;
        temporary   << std::setw( width ) << std::setfill( delim ) << *data;
        return temporary.str();
    } // function to_string 3 argument at less 2 argument

    std::string to_string( int data , unsigned int width , char delim )
    {
        std::basic_stringstream< char > temporary;
        temporary   << std::setw( width ) << std::setfill( delim ) << data;
        return temporary.str();
    } // function to_string 3 argument at less 2 argument
} // namespace interger

} // namespace convert

} // namespace zeabus
