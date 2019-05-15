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

#include    <iostream>

// include all library about arrya
#include    <zeabus/array/handle_array.hpp> 

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
    {
        public:
            DifferentialEquation();

            // size = size of constant_b array and must equal num_b_term
            bool setup_constant_b( double* constant_b , unsigned int size );
            // size = size of constant_a array and must equal num_a_term and first is 1
            bool setup_constant_a( double* constant_a , unsigned int size );
            // 2 functions above will return false if you can't setup data
            // constant_b and constant_a I will start with value 0

            // I think this will help to reduce time of
            void full_fill_buffer( type_buffer value );

            double push( type_buffer data );
            double get_result();

        protected:
            type_buffer input_buffer[ num_b_term ];
            double output_buffer[ num_a_term ];

            double constant_b[ num_b_term ];
            double constant_a[ num_a_term ];

            unsigned int point_input; // I design this variable can have value [0,M)
            unsigned int point_output; // I design this variable can have value [1,N)

            double result;
    }; // object DifferentialEquation

} // namespace filter

} // namespace zeabus

#endif
