// FILE			: count.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 20 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README
//  This file is code in part dynamic_source of zeabus/count.hpp

// REFERENCE

// MACRO SET

namespace zeabus
{

namespace count
{

    template< typename data_type >
    bool compare( data_type data , unsigned int limit_count , bool* count_over )
    {
        static data_type collect_data;
        static unsigned int count_time = -1;
        bool result = true;
        *count_over = true;
        if( count_time == -1 )
        {
            count_time = 0;
            collect_data = data;
        }
        else
        {
            result = (collect_data == data);
            if( result )
            {
                count_time = 0;
            }
            else
            {
                count_time++;
                if( count_time > limit_count )
                {
                    *count_over = false;
                }
            }
        }
        return result 
    }

} // namespace count

} // namespace zeabus
