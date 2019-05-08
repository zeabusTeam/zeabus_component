// FILE         : print_array.cpp
// AUTHOR       : K.Supasan
// CREATE ON    : 2019, APRIL 8
// MAINTAINER   : K.Supasan

#include    <zeabus/print_array.hpp>

namespace zeabus
{

namespace print_array
{

    void integet_type( int* array, unsigned int size, std::string message )
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

} // namespace print_array

} // namespace zeabus
