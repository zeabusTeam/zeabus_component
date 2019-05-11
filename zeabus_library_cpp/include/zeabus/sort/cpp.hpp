// FILE			: cpp.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 10 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README
//  This function is interface to use sort algorithm by cpp

// REFERENCE
//  ref01   : https://en.cppreference.com/w/cpp/algorithm/sort
//  ref02   : https://en.cppreference.com/w/cpp/algorithm/stable_sort

#include    <iostream>

#include    <zeabus/array/copy.hpp>

#include    <algorithm> // include algorithm of cpp

#ifndef _ZEABUS_SORT_CPP_HPP__
#define _ZEABUS_SORT_CPP_HPP__

namespace zeabus
{

namespace sort
{

    template< typename array_type >
    void cpp_sort( array_type* result , unsigned int size );

    template< typename array_type >
    void cpp_sort( array_type* source , array_type* result , unsigned int size );
    
    template< typename array_type >
    void cpp_stable_sort( array_type* result , unsigned int size );

    template< typename array_type >
    void cpp_stable_sort( array_type* source , array_type* result , unsigned int size );

} // namespace sort

} // namespace zeabus

#include    <zeabus/sort/cpp.cpp>
#endif // _ZEABUS_SORT_CPP_HPP__
