// FILE         : cut_off_average.hpp
// AUTHOR       : K.Supasan
// CREATE ON    : 2019, APRIL 8
// MAINTAINER   : K.Supasan

#include    <iostream>

#include    <zeabus/array/print.hpp>

#include    <zeabus/sort/bubble.hpp>

// MACRO_DETAIL 
// _PRINT_FILTER_   :   show array before and after sort 
// _PRINT_PROCESS_  :   show process calculate
// _DEBUG_CODE_     :   show all activity for find debug code

//#define _PRINT_FILTER_
//#define _PRINT_PROCESS_
//#define _DEBUG_CODE_

#ifdef _DEBUG_CODE_
    #define _PRINT_PROCESS_
#endif

#ifdef _PRINT_PROCESS_
    #define _PRINT_FILTER_
#endif

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
            CutOffAverage( unsigned int size_cutoff = 1 );
            
            bool setup_cutoff( unsigned int size_cutoff );
            
            double push( type_buffer data );
 
            double get_result();
 
        protected:
            unsigned int size_cutoff;
            unsigned int size_buffer;
            
        private:
            unsigned int current_point;
            type_buffer sum_buffer;
            double result;
            type_buffer original_buffer[ size ];
            type_buffer temp_buffer[ size ];
    }; // class CutOffAverage

} // namespace filter

} // namespace zeabus

#include    <zeabus/filter/cut_off_average.cpp>

#endif // _ZEABUS_FILTER_CUT_OFF_AVERAGE_HPP__
