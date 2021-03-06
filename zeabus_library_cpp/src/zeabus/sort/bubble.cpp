// FILE         : bubble.cpp
// AUTHOR       : K.Supasan
// CREATE ON    : 2019, APRIL 8
// MAINTAINER   : K.Supasan

#include    <zeabus/sort/bubble.hpp>

namespace zeabus
{

namespace sort
{

    void bubble( int* source , int* result , unsigned int size )
    {
        result[0] = source[0];
#ifdef _PRINT_ARRAY_
        zeabus::print_array::integer_type( source , size , "SOURCE DATA" );
#endif // _PRINT_ARRAY_

        // We will start at order 1. (Indent 0 1 2 3 4 5...)
        for( unsigned int current_point = 1 ; current_point < size ; current_point++ )
        {
            result[current_point] = source[current_point]; // copy original to new array

            for( unsigned int compare_point = current_point; compare_point > 0; compare_point--)
            {
                if( result[compare_point] < result[compare_point-1] )
                {
                    zeabus::swap( &(result[compare_point]) , &(result[compare_point-1]) );
                    // swap data if order compare_point less than order compare_point-1
                }
                else
                {
                    break; // Because before this point already sort
                }
            } // second loop to run compare_point

        } // first loop to run current_point
#ifdef _PRINT_ARRAY_
        zeabus::print_array::integer_type( result , size , "RESULT DATA");
#endif        

    } // bubble function 3 parameter

    void bubble( int* source , unsigned int size )
    {
#ifdef _PRINT_ARRAY_
        zeabus::print_array::integer_type( source , size , "SOURCE DATA" );
#endif // _PRINT_ARRAY_
        for( unsigned int current_point = 1 ; current_point < size ; current_point++ )
        {
            for( unsigned int compare_point = current_point; compare_point > 0; compare_point--)
            {
                if( source[ compare_point ] < source[ compare_point - 1 ] )
                {
                    zeabus::swap( &(source[compare_point]) , &(source[compare_point-1]) );
                }
                else
                {
                    break;
                }
            }  // second lopp to run compare_point
        } // first loop to run current_point
#ifdef _PRINT_ARRAY_
        zeabus::print_array::integer_type( source , size , "AFTER SORT");
#endif        
    } // bubble function 2 parameter

} // namespace sort

} // namespace zeabus
