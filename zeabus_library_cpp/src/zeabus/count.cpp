// FILE			: count.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 20 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README
//  This file this part static source code of zeabus/count.cpp

// REFERENCE

// MACRO SET
//#define _OVER_COUNT_
#define _PROCESS_

#ifdef _PROCESS_
    #define _PROCESS_
#endif

#include    <zeabus/count.hpp>

namespace zeabus
{

namespace count
{

    bool check( bool equal , unsigned int limit_count , bool* count_over )
    {
        static unsigned int count = 0;
        if( equal )
        {
            count = 1;
            *count_over = false;
        }
        else
        {
            count++;
            if( count > limit_count )
            {
                *count_over = true;
#ifdef _OVER_COUNT_
                std::cout   << zeabus::escape_code::bold_yellow << "Count is over on limit is "
                            << limit_count << "\n" << zeabus::escape_code::normal_white;  
#endif
            }
            else
            {
                *count_over = false;
            }
        } // in case not equal
        return equal;
    }

} // namespace count

} // namespace zeabus

