// FILE			: differential_equation_intrfaces.cpp
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

    template< class type_buffer , unsigned int num_b_term , unsigned int num_a_term >
    DifferentialEquationInterfaces< type_buffer, num_b_term, num_a_term>::DifferentialEquationInterfaces()
    {
        zeabus::array::copy::template_type( this->input_buffer , 0.0 , num_b_term );
        zeabus::array::copy::template_type( this->output_buffer , 0.0 , num_a_term );
        this->point_input = 0;
        this->point_output = 0;
    } // function constructor

    template< class type_buffer , unsigned int num_b_term , unsigned int num_a_term >
    bool DifferentialEquationInterfaces< type_buffer ,num_b_term
            , num_a_term >::setup_constant_b( double* constant_b , unsigned int size )
    {
        bool result = true;
        if( size != num_b_term )
        {
            std::cout   << zeabus::escape_code::bold_red << "Fatal by DifferentialEquation : "
                        << zeabus::escape_code::normal_white << "size of term b not equal\n";
            result = false;
        }
        else
        {
            zeabus::array::copy::template_type( this->constant_b , constant_b , size );
        }
        return result;
    }

    template< class type_buffer , unsigned int num_b_term , unsigned int num_a_term >
    bool DifferentialEquationInterfaces< type_buffer , num_b_term 
            , num_a_term >::setup_constant_a( double* constant_a , unsigned int size )
    {
        bool result = true;
        if( size != num_a_term )
        {
            std::cout   << zeabus::escape_code::bold_red << "Fatal by DifferentialEquation : "
                        << zeabus::escape_code::normal_white << "size of term a not equal\n";
        }
        else
        {
            zeabus::array::copy::template_type( this->constant_a , constant_a , size );
        }
    }

    template< class type_buffer , unsigned int num_b_term , unsigned int num_a_term >
    void DifferentialEquationInterfaces< type_buffer, num_b_term
            , num_a_term >::full_fill_buffer( type_buffer value )
    {
        zeabus::array::copy::template_type( this->input_buffer , 1.0*value , num_b_term );
        zeabus::array::copy::template_type( this->output_buffer , 1.0*value , num_a_term );
    }

} // namespace filter

} // namespace zeabus
