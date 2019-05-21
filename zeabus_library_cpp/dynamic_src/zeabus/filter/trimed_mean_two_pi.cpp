// FILE			: trimed_mean_two_pi.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 18 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET
//#define _SHOW_CAL_
//#define _SHOW_MORE_

namespace zeabus
{

namespace filter
{

    template<unsigned int buffer_size , unsigned int trim_size >
    TrimedMean2Pi< buffer_size , trim_size >::TrimedMean2Pi()
    {
        zeabus::array::copy::template_type( this->original_buffer , 0.0 , buffer_size );
        zeabus::array::copy::template_type( this->temp_buffer , 0.0 , buffer_size );
        this->current_point = 0;
        this->result = 0;
    } // contructor of class

    template<unsigned int buffer_size , unsigned int trim_size >
    double TrimedMean2Pi< buffer_size , trim_size >::push( double data )
    {
        this->original_buffer[ this->current_point ] = zeabus::radian::bound( data );
        this->current_point++;
        this->current_point %= buffer_size;
        this->calculate();
        return this->get_result();
    } // function push

    template<unsigned int buffer_size , unsigned int trim_size >
    double TrimedMean2Pi< buffer_size , trim_size >::get_result()
    {
        return this->result;
    } // function get_result

    template<unsigned int buffer_size , unsigned int trim_size >
    void TrimedMean2Pi< buffer_size , trim_size >::calculate()
    {
        zeabus::sort::cpp_stable_sort( this->original_buffer , this->temp_buffer , buffer_size );
#ifdef _SHOW_CAL_
        zeabus::array::print::template_type( this->original_buffer , buffer_size , "Original");
        zeabus::array::print::template_type( this->temp_buffer , buffer_size , "After Sort");
#endif // _SHOW_CAL_
        // First we have to check condition all value have
        bool calculate_type = ( this->temp_buffer[ 0 ] < zeabus::radian::negate_half_pi )
                && ( this->temp_buffer[ buffer_size - 1 ] > zeabus::radian::half_pi );
#ifdef _SHOW_CAL_
        if( calculate_type )
        {
            std::cout   << "Data critical distance min : max are " << (this->temp_buffer)[0]
                        << " : " << (this->temp_buffer)[ buffer_size - 1 ] << "\n";
        }
#endif // _SHOW_CAL_
        if( calculate_type )
        {
            for( unsigned int run = 0 ; run < buffer_size ; run ++ )
            {
                if( (this->temp_buffer)[ run ] < zeabus::radian::negate_half_pi )
                {
                    (this->temp_buffer)[ run ] += zeabus::radian::two_pi;
                }
                else
                {
                    break;
                }
            } // this loop use for rotate < negate half pi by
            zeabus::sort::cpp_stable_sort( this->temp_buffer , buffer_size ); 
#ifdef _SHOW_CAL_
        zeabus::array::print::template_type( this->temp_buffer , buffer_size , "Change temp" );
#endif // _SHOW_CAL_
        } // calculate special case
        this->result = 0;
        unsigned int last_point = buffer_size - trim_size;
        for( unsigned int run = trim_size ; run < last_point ; run++ )
        {
            this->result += (this->temp_buffer)[run];
#ifdef _SHOW_MORE_
            std::cout   << "Add " << this->temp_buffer[run] << " now result is " 
                        << this->result << "\n";
#endif // _SHOW_MORE_
        }
        this->result /= buffer_size - (2*trim_size);
#ifdef _SHOW_CAL_
        std::cout   << "Result is " << this->result;
#endif // _SHOW_CAL_
    } // function calculate 

} // namespace filter

} // namespace zeabus
