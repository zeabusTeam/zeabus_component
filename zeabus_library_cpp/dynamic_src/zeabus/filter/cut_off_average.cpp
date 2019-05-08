// FILE         : cut_off_average.cpp
// AUTHOR       : K.Supasan
// CREATE ON    : 2019, APRIL 8
// MAINTAINER   : K.Supasan

namespace zeabus
{

namespace filter
{

    CutOffAverage< type_buffer , size >::CutOffAverage( unsigned int size_cuttoff )
    {
        for( unsigned int run = 0 ; run < size ; run++ )
        {
            (this->original_buffer)[ run ] = 0;
            (this->temp_buffer)[ run ] = 0;
        }
        this->sum_buffer = 0;
        this->setup_cutoff( size_cuttoff );
    } // function constructor

    bool CutOffAverage< type_buffer , size >::setup_cutoff( unsigned int size_cuttoff )
    {
        bool result = true;
        this->size_cuttoff = size_cuttoff;
        if( this->size_cuttoff > this->size_buffer )
        {
            result = false;
            this->size_cuttoff = (this->size_buffer/2);
        }
        return result;
    } // function setup_cutoff

    void CutOffAverage< type_buffer , size >::push( type_buffer data )
    {
        this->sum_buffer -= (this->original_buffer)[ this->current_point ];
        this->sum_buffer += data;
        (this->original_buffer)[ this->current_point ] = data;
        this->current_point++;
        this->current_point %= this->size_buffer;
#ifdef _PRINT_ARRAY_
        zeabus::print_array::template_type( this->original_buffer , this->size_buffer , "CURRENT BUFFER" );
        std::cout   << "Current sum is " << this->type_buffer;
#endif
    } // function push

    type_buffer CutOffAverage< type_buffer , size >::get_result()
    {
        zeabus::sort::bubble( this->original_buffer , this->temp_buffer );
#ifdef _PRINT_ARRAY_
        zeabus::print_array::template_type( this->original_buffer , this->size_buffer 
                , "ORIGINAL BUFFER") 
        zeabus::print_array::template_type( this->template_type , this->size_buffer 
                , "NEW BUFFER") 
#endif
        type_buffer temp_sum = this->sum_buffer;
        for( unsigned int run = 0 ; run < this->size_cuttoff ; run++ )
        {
            temp_sum -= (this->temp_buffer)[run]; // cutoff minimum
            temp_sum -= (this->temp_buffer)[ this->size_buffer - run - 1 ]; // cutoff maximum
#ifdef _PRINT_PROCESS_
        std::cout   << "run_number " << run << " have order to cut "
                    << (this->temp_buffer)[run] << " and " 
                    << (this->temp_buffer)[ this->size_buffer - run - 1] << "\n";
#endif
        }
        return temp_sum;
    } // function get_result


} // namespace filter

} // namespace zeabus
