// FILE			: time.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 11 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL
//  _PRINT_PROCESS_     : will print every case of set message and result

// README
//  In process to collect log we have to seperate or order file by time.
//  In ros system we can use time by it hard to look
//  That make me make this for manage by implement from ctime header of cpp
//  type_case to assign what pattern do you want
//      1   : YYYY
//      2   : YYYY_MM
//      3   : YYYY_MM_DD
//      4   : YYYY_MM_DD_hh
//      5   : YYYY_MM_DD_hh_mm
//      6   : YYYY_MM_DD_hh_mm_ss

// REFERENCE
//  ref1    : https://en.cppreference.com/w/cpp/chrono/c/tm
//  ref2    : https://en.cppreference.com/w/cpp/chrono/c/gmtime
//  ref3    : https://en.cppreference.com/w/cpp/chrono/c/localtime
//  ref4    : https://stackoverflow.com/questions/35608291/formatting-how-to-convert-1-to-01-2-to-02-3-to-03-and-so-on

// MACRO SET
//#define _PRINT_PROCESS_

#include    <iostream>

#include    <sstream>

#include    <iomanip>

#include    <ctime>

#ifndef _ZEABUS_TIME_HPP__
#define _ZEABUS_TIME_HPP__

namespace zeabus
{

    std::string local_time( unsigned int type_case = 3 );

    std::string gm_time( unsigned int type_case = 3 );

    std::string output_time( std::tm* time , unsigned int type_case );

} // namespace zeabus

#endif //_ZEABUS_TIME_HPP__
