// FILE         : print_array.hpp
// AUTHOR       : K.Supasan
// CREATE ON    : 2019, APRIL 8
// MAINTAINER   : K.Supasan

#include    <iostream>

// README
//  I decision to don't use template pattern because we want this to reduce time of compile
//  That reason make decision to make this library.

#ifndef _ZEABUS_PRINT_ARRAY_HPP__
#define _ZEABUS_PRINT_ARRAY_HPP__

namespace zeabus
{

namespace print_array
{

    void interger_type( int* array , unsigned int size , std::string message = "");

} // namespace print_array

} // namespace zeabus

#endif