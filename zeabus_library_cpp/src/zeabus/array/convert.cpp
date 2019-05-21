// FILE			: convert.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 21 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

#include    <zeabus/array/convert.hpp>

namespace zeabus
{

namespace array
{

namespace convert
{

    std::string to_string( const double* data , const unsigned int size , const char delim )
    {
        std::string message = zeabus::convert::to_string( data[0] );
        for( unsigned int run = 1 ; run < size ; run++ )
        {
            message += delim + zeabus::convert::to_string( data[ run ] );
        }
        return message;
    }

} // namespace convert

} // namespace array

} // namespace zeabus
