// FILE			: copy.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 11 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

#include    <iostream>

#ifndef _ZEABUS_ARRAY_COPY_HPP__
#define _ZEABUS_ARRAY_COPY_HPP__

namespace zeabus
{

namespace array
{

namespace copy
{

    template< typename array_type >
    void template_type( array_type* source , array_type* target , unsigned int size );

} // namespace 

} // namespace array

} // namespace zeabus
#include    <zeabus/array/template_copy.cpp>
#endif
