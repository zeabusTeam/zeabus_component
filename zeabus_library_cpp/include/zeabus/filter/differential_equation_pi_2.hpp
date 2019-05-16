// FILE			: differential_equation_pi_2.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 16 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README
//  Please read general function from DifferentialEquationInterfaces class
//  What is different form DifferentialEquation? Answer is data value
//  This object have purpose to use calculate value in range [0,2*PI] that value have connect
//      in pattern circle.
//  What is distance of 0 and 2*PI? answer is 0
//  I will make some thing to help use caculate this value

// REFERENCE

// MACRO SET

#include    <zeabus/filter/differential_equation_interface.hpp>

#ifndef _ZEABUS_FILTER_DIFFERENTIAL_EQUATION_PI_2_HPP__
#define _ZEABUS_FILTER_DIFFERENTIAL_EQUATION_PI_2_HPP__

namespace zeabus
{

namespace filter
{

    // We don't designed you can assign type of this buffer because
    // in range of [0,2*PI] must use double type only
    template< unsigned int num_b_term, unsigned int num_a_term >
    class DifferntialEquationPI2
            : DifferentialEquationInterfaces< double, num_b_term, num_a_term >
    {
        public:
            DifferntialEquationPI2();

            double push( double data );
            double get_result();

        protected:
            double calculate();
            double temp_input_buffer[ num_b_term ];
            double temp_output_buffer[ num_a_term ];
            
    }; // object DifferntialEquationPI2

} // namespace filter

} // namespace zeabus

#include    <zeabus/filter/differential_equation_pi_2.cpp>
#endif // _ZEABUS_FILTER_DIFFERENTIAL_EQUATION_PI_2_HPP__
