// FILE         : print.cpp
// AUTHOR       : K.Supasan
// CREATE ON    : 2019, APRIL 8
// MAINTAINER   : K.Supasan

#include    <zeabus/array/print.hpp>

namespace zeabus
{

namespace array
{

namespace print
{

    void integer_type( int* array, unsigned int size, std::string message )
    {
        if( message == "")
        {
            std::cout   << "DATA :";
        }
        else
        {
            std::cout   << message << " :";
        }
        for( unsigned int run = 0 ; run < size ; run++ )
        {
            std::cout   << " " << array[run];
        }
        std::cout   << std::endl;
    }

} // namespace print

} // namespace array

} // namespace zeabus
