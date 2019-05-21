// FILE         : trimed_mean.hpp
// AUTHOR       : K.Supasan
// CREATE ON    : 2019, APRIL 8
// MAINTAINER   : K.Supasan

#include    <iostream>

#include    <zeabus/array/print.hpp>

//#include    <zeabus/sort/bubble.hpp>
#include    <zeabus/sort/cpp.hpp>

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

#ifndef _ZEABUS_FILTER_TRIMED_MEAN_HPP__
#define _ZEABUS_FILTER_TRIMED_MEAN_HPP__

namespace zeabus
{

namespace filter
{

    template<class type_buffer, unsigned int size>
    class TrimedMean
    {
        public:
            TrimedMean( unsigned int size_trim = 1 );
            
            bool setup_trim( unsigned int size_trim );
            
            double push( type_buffer data );
 
            double get_result();
 
        protected:
            unsigned int size_trim;
            unsigned int size_buffer;
            
        private:
            unsigned int current_point;
            type_buffer sum_buffer;
            double result;
            type_buffer original_buffer[ size ];
            type_buffer temp_buffer[ size ];
    }; // class TrimMean

} // namespace filter

} // namespace zeabus

#include    <zeabus/filter/trimed_mean.cpp>

#endif // _ZEABUS_FILTER_TRIMED_MEAN_HPP__
