// FILE			: radian.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 18 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

#include    <zeabus/radian.hpp>

namespace zeabus
{

namespace radian
{

    double bound( double number )
    {
        double answer = number;
        while( true )
        {
            if( answer > pi )
            {
                answer += negate_two_pi;
            }
            else if( answer < -pi )
            {
                answer += two_pi;
            }
            else
            {
                break;
            }
        }
        return answer;
    } // function bound parameter is value

    void bound( double* number )
    {
        *number = bound( *number );
    } // function bound pass by pointer value

    double different( double start , double target )
    {
        double answer = target - start;
        bound( &answer );
        return answer;
    } // function different pass by two value

    double different( double* start , double* target )
    {
        double answer = (*target) - (*start);
        bound( &answer );
        return answer;
    } // function different pass by pointer double

    void different( double* start , double* target , double* result )
    {
        *result = different( start , target );
    } // function different pass & get answer by pointer double

} // namespace radian

} // namespace zeabus
