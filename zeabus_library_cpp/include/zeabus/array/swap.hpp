// FILE			: swap.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 11 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

#include    <iostream>

#ifndef _ZEABUS_ARRAY_SWAP_HPP__
#define _ZEABUS_ARRAY_SWAP_HPP__

namespace zeabus
{

namespace array
{

namespace swap
{
    
    template< typename array_type >
    void template_type( array_type* first, array_type* second, unsigned int size);

} // namespace swap

} // namespace array

} // namespace zeabus
#include    <zeabus/array/template_swap.hpp>
#endif
