// FILE			: differential_equation.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 15 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

namespace zeabus
{

namespace filter
{

    template< class type_buffer , unsigned int num_b_term , unsigned int num_a_term >
    DifferentialEquation< type_buffer, num_b_term, num_a_term>::DifferentialEquation()
            : DifferentialEquationInterfaces()
    {
        ;
    } // function constructor

    template< class type_buffer , unsigned int num_b_term , unsigned int num_a_term >
    double DifferentialEquation<type_buffer, num_b_term, num_a_term >::push( type_buffer data )
    {
        this->input_buffer[this->point_input] = data; // first time we must collect data
        this->point_output--; // and delay output to one so this point we collect answer
        (this->output_buffer)[this->point_output] = this->calculate(); // calculate and save
        this->point_input++; // Finish calculate move to next point
        return this->get_result();
    }

    template< class type_buffer , unsigned int num_b_term, unsigned int num_a_term >
    double DifferentialEquation< type_buffer, num_b_term, num_a_term >::get_result()
    {
        return (this->output_buffer)[this->point_output];
    }

    // This function will have connectioned to 2 variable you must to know
    //      this->point_input   : This data must point to last input data
    //      this->point_output  : This data must point to member will replace by result of 
    //                          calculation from input active
    template< class type_buffer, unsigned int num_b_term , unsigned int num_a_term >
    double DifferentialEquation<type_buffer, num_b_term, num_a_term >::calculate()
    {
        double result = 0.0;
        for( unsigned int run = 0 ; run < num_b_term ; run++ )
        {
            result += ( (this->input_buffer)[ (this->point_input + run )%num_b_term ] 
                        * (this->constant_b)[run] );
        }
        for( unsigned int run = 1 ; run < num_a_term ; run++ )
        {
            result += ( (this->output_buffer)[ (this->point_output + run)%num_a_term ]
                        * (this->constant_a)[run] );
        }
        return result;
    }
} // namespace zeabus
 
} // namespace filter
