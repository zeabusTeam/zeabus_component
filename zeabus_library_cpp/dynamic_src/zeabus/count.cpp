// FILE			: count.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 20 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README
//  This file is code in part dynamic_source of zeabus/count.hpp

// REFERENCE

// MACRO SET
//#define _OVER_COUNT_
//#define _PROCESS_

namespace zeabus
{

namespace count
{

    template< typename data_type >
    bool compare( data_type data , unsigned int limit_count , bool* count_over )
    {
        static data_type collect_data;
        static unsigned int count_time = 0xffffffff;
        bool result = true;
        *count_over = false; // if over will return true
        if( count_time == 0xffffffff )
        {
            count_time = 0;
            collect_data = data;
#ifdef _PROCESS_
                std::cout   << "Reset data count is " << count_time << std::endl;
#endif
        }
        else
        {
            result = (collect_data != data);
            if( result )
            {
                collect_data = data;
                count_time = 0;
#ifdef _PROCESS_
                std::cout   << "NEW data count is " << count_time << std::endl;
#endif
            }
            else
            {
                count_time++;
#ifdef _PROCESS_
                std::cout   << "Same data count is " << count_time << std::endl;
#endif
                if( count_time > limit_count )
                {
#ifdef _OVER_COUNT_
                    std::cout   << zeabus::escape_code::bold_yellow 
                                << "Count is over on limit is "
                                << limit_count << "\n" << zeabus::escape_code::normal_white;  
#endif
                    *count_over = true;
                }
            }
        }
        return result; 
    }

} // namespace count

} // namespace zeabus
