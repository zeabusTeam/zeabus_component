// FILE			: count.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 20 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README
//  This file this part static source code of zeabus/count.cpp

// REFERENCE

// MACRO SET

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
            count = 0;
            *count_over = false;
        }
        else
        {
            count++;
            if( count < limit_count )
            {
                *count_over = true;
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

