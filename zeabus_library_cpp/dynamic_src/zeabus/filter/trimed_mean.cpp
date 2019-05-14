// FILE         : trimed_mean.cpp
// AUTHOR       : K.Supasan
// CREATE ON    : 2019, APRIL 8
// MAINTAINER   : K.Supasan

namespace zeabus
{

namespace filter
{

    template< class type_buffer , unsigned int size >
    TrimedMean< type_buffer , size >::TrimedMean( unsigned int size_trim )
    {
#ifdef _DEBUG_CODE_
        std::cout   << "Before init value of buffer\n";
#endif
        for( unsigned int run = 0 ; run < size ; run++ )
        {
            (this->original_buffer)[ run ] = 0;
            (this->temp_buffer)[ run ] = 0;
        }
#ifdef _DEBUG_CODE_
        std::cout   << "Finish init value of buffer\n";
#endif
        this->size_buffer = size;
        this->sum_buffer = 0;
        this->current_point = 0;
        this->setup_trim( size_trim );
    } // function constructor

    template< class type_buffer , unsigned int size >
    bool TrimedMean< type_buffer , size >::setup_trim( unsigned int size_trim )
    {
#ifdef _DEBUG_CODE_
        std::cout   << "Want to setup_trim\n";
#endif
        bool result = true;
        this->size_trim = size_trim;
        if( this->size_trim > this->size_buffer )
        {
            result = false;
            this->size_trim = (this->size_buffer/2);
        }
        return result;
    } // function setup_cutoff

    template< class type_buffer , unsigned int size >
    double TrimedMean< type_buffer , size >::push( type_buffer data )
    {
#ifdef _DEBUG_CODE_
        std::cout   << "Before function to push data\n";
#endif
        this->sum_buffer -= (this->original_buffer)[ this->current_point ];
        this->sum_buffer += data;
        (this->original_buffer)[ this->current_point ] = data;
        this->current_point++;  
        this->current_point %= this->size_buffer;
#ifdef _PRINT_FILTER_
        zeabus::array::print::template_type( this->original_buffer , this->size_buffer , "CURRENT BUFFER" );
        std::cout   << "Current sum is " << this->size_buffer << std::endl;
#endif
    // Next part of calculate output of filter
        zeabus::sort::cpp_stable_sort( this->original_buffer , this->temp_buffer 
                , this->size_buffer);
#ifdef _PRINT_FILTER_
        zeabus::array::print::template_type( this->original_buffer , this->size_buffer 
                , "ORIGINAL BUFFER");
        zeabus::array::print::template_type( this->temp_buffer , this->size_buffer 
                , "NEW BUFFER");
#endif
        this->result = 1.0 * this->sum_buffer;
        for( unsigned int run = 0 ; run < this->size_trim ; run++ )
        {
            this->result -= (this->temp_buffer)[run]; // trim minimum
            this->result -= (this->temp_buffer)[ this->size_buffer - run - 1 ]; // trim maximum
#ifdef _PRINT_PROCESS_
        std::cout   << "run_number " << run << " have order to cut "
                    << (this->temp_buffer)[run] << " and " 
                    << (this->temp_buffer)[ this->size_buffer - run - 1] << "\n";
#endif
        }
        return ( this->result ) / (this->size_buffer - (this->size_trim * 2) );
    } // function push

    template< class type_buffer , unsigned int size >
    double TrimedMean< type_buffer , size >::get_result()
    {
        return this->result;
    } // function get_result


} // namespace filter

} // namespace zeabus
