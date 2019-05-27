// FILE			: control_error.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 27 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README
//  Version is 0.5.0
//  This fuzzy will have input only one and will turn about force but in calcualted
//      we use relative pattern to calculate the output

// REFERENCE
//  ref01   : http://www.cplusplus.com/reference/array/array/fill/

// MACRO SET

// MACRO CONDITION

#include    <cmath>

#include    <array>
// function will use is fabs , copysign

#include    <iostream>

#ifndef _ZEABUS_FUZZY_CONTROL_ERROR_HPP__
#define _ZEABUS_FUZZY_CONTROL_ERROR_HPP__

namespace zeabus
{

namespace fuzzy
{

    // Use buffer_size to assign about time to change output buffer
    template< unsigned int buffer_size >
    class ControlError
    {
        public:
            
            ControlError( double offset = 0 );

            void set_relative_value( double low , double medium , double high );

            // Below function is important to use because we want parameter to set fuzzy system
            void set_diff_zero( double ok_error = 0.01 );
            // This will mean when we want zero
            void set_offset_force( double offset = 0 );
            // This will use to assign range of error
            void set_error_range( double low , double medium , double high );
            // This will use to assing range of different buffer
            void set_diff_range( double low , double medium , double high );
            // This will use to assign range of force or output will use in output_condition
            void set_force_range( double low , double medium , double high );
            // function clear system will clear all buffer and set output to offset
            void clear_system();
            // function push will give interface of system to input and get output
            double push( double error );
            // function get_result will return output for you
            double get_result();

        protected:
            // function output condition we have input is this->output and will edit value
            void output_condition();
            void rule_condition();
            void fuzzy_condition( double error );

        private:
            // 3 line belows will about output force
            double offset;
            double output;
            double buffer_output;
            // 3 line belows will about buffer of different
            unsigned int point_element;
            double sum_buffer;
            std::array< double , buffer_size > buffer_different;
            // 3 line belows will about collect fuzzy rule
            std::array< double , 3 > error_range;
            std::array< double , 3 > diff_range;
            std::array< double , 3 > force_range;
            // use rule_condition to updated 2 below variable
            signed char error_fuzzy;
            signed char diff_fuzzy; 
            signed char result_fuzzy;
            

    }; // class object ControlError

} // namespace fuzzy

} // namespace zeabus

#include    <zeabus/fuzzy/control_error.cpp>

#endif // _ZEABUS_FUZZY_CONTROL_ERROR_HPP__
