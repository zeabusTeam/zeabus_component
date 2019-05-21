// FILE			: trimed_mean_two_pi.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 18 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL
//  _SHOW_MORE_ : This macro will auto include _SHOW_CAL_ but will show data to use average
//  _SHOW_CAL_  : This macro will help you to see calculate process

// README
//  Thie style similar trimed_mean code of zeabus_library but this code will solve problem for 
//  find average or mean of angle circle.
//  Please read K.Supasan 20190517dailynote.pdf to undestand thinking of author
//  This file we will use domain in (-pi,pi) because we this value is base from imu connection

// REFERENCE

// MACRO SET
//#define _SHOW_CAL_
//#deifne _SHOW_MORE_

#ifdef _SHOW_MORE_
    #define _SHOW_CAL_
#endif // _SHOW_MORE_

#include    <iostream>

#include    <zeabus/radian.hpp>

#include    <zeabus/array/handle_array.hpp>

#include    <zeabus/sort/cpp.hpp>

#ifndef _ZEABUS_FILTER_TRIMED_MEAN_TWO_PI_HPP
#define _ZEABUS_FILTER_TRIMED_MEAN_TWO_PI_HPP

namespace zeabus
{

namespace filter
{

    // This code we design two get number parameter same past object 
    //  but different because we receive for size only
    template<unsigned int buffer_size , unsigned int trim_size >
    class TrimedMean2Pi
    {
        public:
            TrimedMean2Pi();

            // two funtion below will reture current data
            double push( double data );
            double get_result();
        
        protected:
            // this function will updated current of filter to result variable
            void calculate();

        private:
            // collect raw input data in domain [-pi , pi]
            double original_buffer[ buffer_size ];
            // collect temp in put for calculate
            double temp_buffer[ buffer_size ];
            double result;
            unsigned int current_point;
    };

} // namespace filter

} // namespace zeabus
#include    <zeabus/filter/trimed_mean_two_pi.cpp>
#endif // _ZEABUS_FILTER_TRIMED_MEAN_TWO_PI_HPP
