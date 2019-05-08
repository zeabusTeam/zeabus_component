// FILE         : cut_off_average.hpp
// AUTHOR       : K.Supasan
// CREATE ON    : 2019, APRIL 8
// MAINTAINER   : K.Supasan

#include    <iostream>

#include    <zeabus/print_array.hpp>

#include    <zeabus/sort/bubble.hpp>

// MACRO_DETAIL 
// _PRINT_ARRAY_    :   

#ifndef _ZEABUS_FILTER_CUT_OFF_AVERAGE_HPP__
#define _ZEABUS_FILTER_CUT_OFF_AVERAGE_HPP__

namespace zeabus
{

namespace filter
{

    template<class type_buffer, unsigned int size>
    class CutOffAverage
    {
        public:
            CutOffAverage( unsigned int size_cutoff = 2 );
            
            bool setup_cutoff( unsigned int size_cutoff );
            
            void push( type_buffer data );
 
            type_buffer get_result();
 
        protected:
            unsigned int size_cutoff;
            unsigned int size_buffer;
            
        private:
            unsigned int current_point;
            type_buffer sum_buffer;
            type_buffer original_buffer[ size ];
            type_buffer temp_buffer[ size ];
    }; // class CutOffAverage

} // namespace filter

} // namespace zeabus

#include    <zeabus/filter/cut_off_average.cpp>

#endif // _ZEABUS_FILTER_CUT_OFF_AVERAGE_HPP__
