// FILE			: cpp.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 11 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

namespace zeabus
{

namespace sort
{

    template< typename array_type >
    void cpp_sort( array_type* result , unsigned int size )
    {
        std::sort( result , result + size );
    }

    template< typename array_type >
    void cpp_sort( array_type* source , array_type* result , unsigned int size )
    {
        zeabus::array::copy( source , result , size );
        zeabus::sort::cpp_sort( result , size );
    }

    template< typename array_type >
    void cpp_stable_sort( array_type* result , unsigned int size )
    {
        std::stable_sort( result , result + size );
    }

    template< typename array_type >
    void cpp_stable_sort( array_type* source , array_type* result , unsigned int size )
    {
        zeabus::array::copy( source , result , size );
        zeabus::sort::cpp_stable_sort( result , size );
    }

} // namespace sort

} // namespace zeabus
