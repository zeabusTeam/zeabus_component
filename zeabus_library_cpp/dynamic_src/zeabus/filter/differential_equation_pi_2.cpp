// FILE			: differential_equation_pi_2.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 16 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

namespace zeabus
{

namespace filter
{

    template< unsigned int num_b_term, unsigned int num_a_term >
    DifferentialEquationPI2< num_b_term , num_a_term >::DifferentialEquationPI2()
            : DifferentialEquationInterfaces()
    {
        zeabus::array::copy::template_type( this->temp_input_buffer , 0.0 , num_b_term );
        zeabus::array::copy::template_type( this->temp_output_buffer , 0.0 , num_a_term );
    } // function constructor 

    template< unsigned int num_b_term, unsigned int num_a_term >
    double DifferentialEquationPI2< num_b_term ,num_a_term >::push(double data )
    {

    } // function push

    template< unsigned int num_b_term, unsigned int num_a_term >
    double DifferentialEquationPI2< num_b_term, num_a_term >::get_result()
    {

    } // function get_result

    template< unsigned int num_b_term, unsigned int num_a_term >
    double DifferentialEquationPI2< num_b_term, num_a_term >::calculate()
    {
        
    } // function calculate

} // namespace filter

} // namespace zeabus
