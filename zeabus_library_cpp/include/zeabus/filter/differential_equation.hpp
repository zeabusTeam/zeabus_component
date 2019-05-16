// FILE			: differential_equation.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 15 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL


// README <template pattern dynamic_src>
//  What is differential_equation filter? We know in of about b_k a_l term
//  Can use in matlab by function filter
//  General Term is
//          y[n] = ( Sumation< l = [1,N] > a_l*y[n-l] ) + ( Sumation< k = [0,M]> b_k*y[n-k] )

// REFERENCE

// MACRO SET

#include    <zeabus/filter/differential_equation_interfaces.hpp>

#ifndef _ZEABUS_FILTER_DIFFERENTIAL_EQUATION_HPP__
#define _ZEABUS_FILTER_DIFFERENTIAL_EQUATION_HPP__

namespace zeabus
{

namespace filter
{

    // What about parameter num_b_term
    //      This parameter will decision what size of input array and use in to M term of
    //      equation on line 13
    // What about parameter num_a_term
    //      This parameter will decision what size of output array and use to N term of 
    //      equation on line 13
    // What rule to setup parameter num_a_term and num_b_term? Please thinking you want to use
    //      function filter in MATLAB you have to setup same parameter of that.
    template< class type_buffer , unsigned int num_b_term , unsigned int num_a_term>
    class DifferentialEquation 
            : DifferentialEquationIntrfaces< type_buffer , num_b_term , num_a_term >
    {
        public:
            DifferentialEquation();

            double push( type_buffer data );
            double get_result();

        protected:
            double calculate();
    }; // object DifferentialEquation

} // namespace filter

} // namespace zeabus

#include    <zeabus/filter/differential_equation.cpp>

#endif // _ZEABUS_FILTER_DIFFERENTIAL_EQUATION_HPP__
