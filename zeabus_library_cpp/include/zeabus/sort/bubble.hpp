// FILE         : bubble.hpp
// AUTHOR       : K.Supasan
// CREATE ON    : 2019, APRIL 8
// MAINTAINER   : K.Supasan

#include    <iostream>

#include    <zeabus/swap.hpp>

#include    <zeabus/print_array.hpp>

// MACRO_DETAIL 
// _PRINT_ARRAY_        : This macro will print array after and before source array

// REFERENCE
//  ref01   : https://www.geeksforgeeks.org/bubble-sort/

// README PLEASE
//  Our function don't have to protected about size of array to access data.
//  Please sure your array have lenght more or equal size arguments.

#ifndef _ZEABUS_SORT_BUBBLE_HPP__
#define _ZEABUS_SORT_BUBBLE_HPP__

namespace zeabus
{

namespace sort
{

    // WARNING THIS SORT DON'T MALLOC YOU BUFFER/ARRAY.
    //  please sure your have size can access data in your array

    void bubble( int* source , int* result ,  unsigned int size );

    void bubble( int* source , unsigned int size );

} // namespace sort

} // namespace zeabus

#endif // _ZEABUS_SORT_BUBBLE_HPP__
