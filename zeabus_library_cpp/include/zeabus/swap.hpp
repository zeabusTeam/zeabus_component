// FILE         : swap.hpp
// AUTHOR       : K.Supasan
// CREATE ON    : 2019, APRIL 8
// MAINTAINER   : K.Supasan

#include    <iostrem>

#ifndef _ZEABUS_SWAP_HPP_
#define _ZEABUS_SWAP_HPP_

namespace zeabus
{

    template< typename type_value >
    void swap( type_value* source , type_value* destination );

}

#include    <zeabus/swap.cpp>

#endif // _ZEABUS_SWAP_HPP_
