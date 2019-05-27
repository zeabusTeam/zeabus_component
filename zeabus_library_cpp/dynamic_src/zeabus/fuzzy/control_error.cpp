// FILE			: control_error.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 27 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

namespace zeabus
{

namespace fuzzy
{

    template< unsigned int buffer_size >
    ControlError::ControlError( double offset )
    {
        this->set_offset_force( offset );
    }

    template< unsigned int buffer_size >
    void ControlError::clear_system()
    {
        this->previous_error = 0;
        this->sum_buffer = 0;
        this->buffer_output = this->offset;
        this->point_element = 0;
        (this->buffer_different).fill( 0 ); 
    } // function clear_system 

    template< unsigned int buffer_size >
    void ControlError::set_offset_force( double offset )
    {
        this->offset = offset;
    } // function set_offset_force

    template< unsigned int buffer_size >
    void ControlError::set_error_range( double low, double medium, double high )
    {
        (this->error_range)[0] = fabs(slow);
        (this->error_range)[1] = fabs(medium);
        (this->error_range)[2] = fabs(high;)
    } // function set_error_range

    template< unsigned int buffer_size >
    void ControlError::set_diff_range( double low, double medium , double high )
    {
        (this->diff_range)[0] = fabs(low);
        (this->diff_range)[1] = fabs(medium);
        (this->diff_range)[2] = fabs(high);
    } // function set_diff_range

    template< unsigned int buffer_size >
    void ControlError:;set_force_range( double low , double medium , double high )
    {
        (this->force_range)[0] = fabs( low );
        (this->force_range)[1] = fabs( meidum );
        (this->force_range)[2] = fabs( high );
    }

    template< unsigned int buffer_size >
    double ControlError::push( double error )
    {
        // process about manage buffer
        double diff_error = error - this->previous_error;
        this->sum_buffer -= (this->buffer_different)[ this->point_element ];
        (this->buffer_different)[ this->point_element ] = diff_error;
        this->sum_buffer += diff_error;
        this->previous_error = error;
        // next process condition time to calculate or not
        this->point_element += 1;
        if( this->point_element == buffer_size )
        {
            this->point_element = 0;
            this->fuzzy_condition( error );
        }
        return this->get_result(); 
    } // function push

    template< unsigned int buffer_size >
    double ControlError::get_result()
    {
        return this->output;
    } // function get_result

    // Next is part about protedced function
    template< unsigned int buffer_size >
    double ControlError::output_condition()
    {
        
    } // function output_condition

    template< unsigned int buffer_size >
    double ControlError::rule_condition()
    {
    } // function rule_condition

    template< unsigned int buffer_size >
    double ControlError::fuzzy_condition( double error )
    {
        double temp_data;
        // get value to error_fuzzy
        temp_data = fabs( error );
        if( temp_data > error_range[2] ) this->error_fuzzy = copysign( 3 , error );
        else if( temp_data > error_range[1] ) this->error_fuzzy = copysign( 2 , error );
        else if( temp_data > error_range[0] ) this->error_fuzzy = copysign( 1 , error );
        else this->error_fuzzy = 0;
        // get value to diff_fuzzy
        temp_data = fabs( this->sum_buffer / buffer_size );
        if( temp_data > this->diff_range[2] ) 
        {
            this->diff_fuzzy = copysign( 3 ,this->sum_buffer );
        }
        else if( temp_data > this->diff_range[1] )
        {
            this->diff_fuzzy = copysign( 2 , this->sum_buffer );
        }
        else if( temp_data > this->diff_range[0] ) 
        {
            this->diff_fuzzy = copysign( 1 , this->sum_buffer );
        }
        else this->diff_fuzzy = 0;
        
    } // function fuzzy_condition

    
} // namespace fuzzy

} // namespace zeabus
