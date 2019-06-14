// FILE			: control_error.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 27 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL
// Pleasr read on header file location is 
//      package zeabus_library_cpp path is include/zeabus/fuzzy/control_error.hpp

// README

// REFERENCE

// MACRO SET
//#define _SHOW_DATA_
//#define _SHOW_OUTPUT_CONDITION_
//#define _SHOW_RULE_TABLE_

// MACRO CONDITION

namespace zeabus
{

namespace fuzzy
{

    template< unsigned int buffer_size >
    ControlError< buffer_size >::ControlError( double offset )
    {
        this->set_offset_force( offset );
    }

    template< unsigned int buffer_size >
    void ControlError< buffer_size >::clear_system()
    {
        this->reverse_state = false;
        this->previous_error = 0;
        this->sum_buffer = 0;
        this->buffer_output = this->offset;
        this->point_element = 0;
        (this->buffer_different).fill( 0 ); 
        this->previous_error_fuzzy = 0;
    } // function clear_system 

    template< unsigned int buffer_size >
    void ControlError< buffer_size >::set_offset_force( double offset )
    {
        this->offset = offset;
    } // function set_offset_force

    template< unsigned int buffer_size >
    void ControlError< buffer_size >::set_error_range( double low, double medium, double high )
    {
        (this->error_range)[0] = fabs(low);
        (this->error_range)[1] = fabs(medium);
        (this->error_range)[2] = fabs(high);
    } // function set_error_range

    template< unsigned int buffer_size >
    void ControlError< buffer_size >::set_diff_range( double low, double medium , double high )
    {
        (this->diff_range)[0] = fabs(low);
        (this->diff_range)[1] = fabs(medium);
        (this->diff_range)[2] = fabs(high);
    } // function set_diff_range

    template< unsigned int buffer_size >
    void ControlError< buffer_size >::set_force_range( double low , double medium , double high )
    {
        (this->force_range)[0] = fabs( low );
        (this->force_range)[1] = fabs( medium );
        (this->force_range)[2] = fabs( high );
    }

    template< unsigned int buffer_size >
    void ControlError< buffer_size >::set_relative_value( double low , double medium 
            , double high )
    {
        (this->relative_value)[0] = 0.0;
        (this->relative_value)[1] = fabs( low );
        (this->relative_value)[2] = fabs( medium );
        (this->relative_value)[3] = fabs( high );
    }

    template< unsigned int buffer_size >
    double ControlError< buffer_size >::push( double error )
    {
        // process about manage buffer
        double diff_error = this->previous_error - error;
        this->sum_buffer -= (this->buffer_different)[ this->point_element ];
        (this->buffer_different)[ this->point_element ] = diff_error;
        this->sum_buffer += diff_error;
        this->previous_error = error;
        // next process condition time to calculate or not
        this->point_element += 1;
        if( this->point_element == buffer_size )
        {
#ifdef _SHOW_DATA_
            std::cout   << "-----------------------------------------------\n";
            std::cout   << "New result and get error is " << error << "\n";
#endif
            this->fuzzy_condition( error );
            this->point_element = 0;
        }
        return this->get_result(); 
    } // function push

    template< unsigned int buffer_size >
    double ControlError< buffer_size >::get_result()
    {
        return this->output;
    } // function get_result

    // Next is part about protedced function
    template< unsigned int buffer_size >
    void ControlError< buffer_size >::output_condition()
    {
        // Above this line will be have condition to estimate or june output data
        // I will interest about error_fuzzy to use and make condition
#ifdef _SHOW_OUTPUT_CONDITION_
        std::cout   << "\tOutput Condition Function : previous , current " 
                    <<  this->previous_error_fuzzy << " , " << this->error_fuzzy << "\n";
#endif
        this->reverse_state = false;
        if( this->error_fuzzy == 0 )
        {
            if( abs( this->buffer_output ) < this->force_range[0] 
                    || ( this->previous_error_fuzzy == 0 ) )
            {
                this->output = this->offset;
#ifdef _SHOW_OUTPUT_CONDITION_
                std::cout   << zeabus::escape_code::bold_yellow 
                            << "\t\tOutput set force to offset and result is " << this->output 
                            << "\n" << zeabus::escape_code::normal_white;
#endif
            }
            else
            {
                this->output = -1 * this->buffer_output + this->offset;
#ifdef _SHOW_OUTPUT_CONDITION_
                std::cout   << zeabus::escape_code::bold_margenta
                            << "\t\tOutput set reverse force and result is " << this->output
                            << "\n" << zeabus::escape_code::normal_white;
#endif
                this->reverse_state = true;
            }
        }
        else if( abs( this->previous_error_fuzzy ) > abs( this->error_fuzzy ) )
        {
            this->output = copysign( this->force_range[ abs(this->error_fuzzy) ] 
                    , this->error_fuzzy );
            if( ( this->output * this->offset ) < 0 )
            {
                this->output += this->offset;
            }
#ifdef _SHOW_OUTPUT_CONDITION_
            std::cout   << zeabus::escape_code::bold_yellow
                        << "\t\tOutput set maximum force of range with offset force is "
                        << this->output <<"\n"
                        << zeabus::escape_code::normal_white;
#endif
        }
        else
        {
            ; // normal case of fuzzy
#ifdef _SHOW_OUTPUT_CONDITION_
            std::cout   << zeabus::escape_code::bold_blue
                        << "\t\tNormal output and result is "
                        << this->output <<"\n"
                        << zeabus::escape_code::normal_white;
#endif
        }
        if( ( abs(this->output) > (this->force_range)[2] ) && (!this->reverse_state))
        {
            // In this case we have to check about sign bit of offset and output
            if(  ( ( this->output * this->offset ) < 0 ) && ( this->offset != 0 ) )
            {
                this->output = copysign( this->offset , this->output );
            } // this mean your data have opposite sign
            else
            {
                this->output = copysign( (this->force_range)[2] , this->output );
            }
        }
        this->previous_error_fuzzy = this->error_fuzzy;
        if( this->reverse_state )
        {
            this->buffer_output = this->offset;
        }
        else
        {
            this->buffer_output = this->output;
        }
    } // function output_condition

    template< unsigned int buffer_size >
    void ControlError< buffer_size >::rule_condition()
    {
#ifdef _SHOW_RULE_TABLE_
        for( unsigned int run_y = 0 ; run_y < 7 ; run_y++ )
        {
            for( unsigned int run_x = 0 ; run_x < 7 ; run_x++ )
            {
                std::cout   << "\t" << (this->rule_table)[ run_y ][ run_x ];
            }
            std::cout   << "\n";
        }
        std::cout   << "We use order ( " << this->error_fuzzy + 3 
                    << " , " << this->diff_fuzzy + 3 << " )\n"; 
#endif
        this->result_fuzzy = (this->rule_table)[ this->diff_fuzzy + 3 ][ this->error_fuzzy + 3 ];
#ifdef _SHOW_DATA_
        std::cout   << "Fuzzy summary result is " << this->result_fuzzy << "\n";
#endif
        double temp_data = (this->relative_value)[ fabs( this->result_fuzzy ) ];
        double temp_sign = 1.0*this->result_fuzzy;
        double temp_result = copysign( temp_data  , temp_sign );
#ifdef _SHOW_DATA_
        std::cout   << "Fuzzy summary plus force is " << temp_result <<"\n";
#endif
        this->output = this->buffer_output + temp_result;
        this->output_condition();
#ifdef _SHOW_DATA_
        std::cout   << "Result is  " << this->get_result() << "\n";
#endif
    } // function rule_condition

    template< unsigned int buffer_size >
    void ControlError< buffer_size >::set_rule_condition( 
            std::array<std::array< int , 7 > , 7 >* rule )
    {
        this->rule_table = *rule;
#ifdef _SHOW_RULE_TABLE_
        for( unsigned int run_y = 0 ; run_y < 7 ; run_y++ )
        {
            for( unsigned int run_x = 0 ; run_x < 7 ; run_x++ )
            {
                std::cout   << this->rule_table[ run_y ][ run_x ] << "\t";
            }
            std::cout   << "\n";
        }
#endif
    }

    template< unsigned int buffer_size >
    void ControlError< buffer_size >::fuzzy_condition( double error )
    {
        double temp_data;
#ifdef _SHOW_DATA_
        std::cout   << "Input error " << error << "\n";
#endif
        // get value to error_fuzzy
        temp_data = fabs( error );
        if( temp_data > error_range[2] ) this->error_fuzzy = copysign( 3 , error );
        else if( temp_data > error_range[1] ) this->error_fuzzy = copysign( 2 , error );
        else if( temp_data > error_range[0] ) this->error_fuzzy = copysign( 1 , error );
        else this->error_fuzzy = 0;
#ifdef _SHOW_DATA_
        std::cout   << "\tError is " << error << " and rule get " 
                    << ( this->error_fuzzy ) << "\n";
#endif
        // get value to diff_fuzzy
        temp_data = fabs( this->sum_buffer );
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
#ifdef _SHOW_DATA_
        std::cout   << "\tsum_buffer is " << temp_data << " and rule get " 
                    << ( this->diff_fuzzy ) << "\n";
#endif
        this->rule_condition(); 
    } // function fuzzy_condition

    
} // namespace fuzzy

} // namespace zeabus
