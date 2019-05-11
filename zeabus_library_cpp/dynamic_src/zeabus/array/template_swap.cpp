// FILE			: template_swap.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 11 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

namespace zeabus
{

namespace array
{

namespace swap
{

    template< typename array_type >
    void template_type( array_type** first , array_type** second, unsigned int size )
    {
        array_type** temporary;
        *temporary = *first;
        *first = *second;
        *second = *temporary;
    } // function template_type

} // namespace swap

} // namespace array

} // namespace zeabus
