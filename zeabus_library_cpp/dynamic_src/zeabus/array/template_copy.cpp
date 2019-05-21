// FILE			: template_copy.cpp
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

namespace copy
{

    template< typename array_type >
    void template_type( array_type* source, array_type* target, unsigned int size )
    {
        for( unsigned int run = 0 ; run < size ; run++ )
        {
            target[ run ] = source[ run ];
        }
    } // function template_type

    template< typename array_type >
    void template_type( array_type* source , array_type value , unsigned int size )
    {
        for( unsigned int run = 0 ; run < size ; run++ )
        {
            source[ run ] = value;
        }
    } // function template_type

} // namesapce copy

} // namespace array

} // namespace zeabus
