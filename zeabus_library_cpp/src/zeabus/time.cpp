// FILE			: time.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 11 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET
//#define _PRINT_PROCESS_
//#define _PRINT_TEMPORARY_

#include    <zeabus/time.hpp>

namespace zeabus
{

    std::string local_time( unsigned int type_case )
    {
        std::time_t temp = std::time( nullptr );
        return output_time( std::localtime( &temp ) , type_case );
    }

    std::string gm_time( unsigned int type_case )
    {
        std::time_t temp = std::time( nullptr );
        return output_time( std::gmtime( &temp ) , type_case );
    }

    std::string output_time( std::tm* time , unsigned int type_case )
    {
        const static char delim = '0';
        std::string result = "";
        switch( type_case )
        {
            case 6  :
                result = "_" + zeabus::convert::integer::to_string( time->tm_sec , 2 , delim) 
                        + result;
#ifdef _PRINT_PROCESS_
                std::cout   << "case 6 : " << result << "\n";
#endif // _PRINT_PROCESS_ 
            case 5  :
                result = "_" + zeabus::convert::integer::to_string( time->tm_min, 2 , delim ) 
                        + result;
#ifdef _PRINT_PROCESS_
                std::cout   << "case 5 : " << result << "\n";
#endif // _PRINT_PROCESS_ 
            case 4  :
                result = "_" + zeabus::convert::integer::to_string( time->tm_hour , 2 , delim ) 
                        + result;
#ifdef _PRINT_PROCESS_
                std::cout   << "case 4 : " << result << "\n";
#endif // _PRINT_PROCESS_ 
            case 3  :
                result = "_" + zeabus::convert::integer::to_string( time->tm_mday , 2 , delim ) 
                        + result;
#ifdef _PRINT_PROCESS_
                std::cout   << "case 3 : " << result << "\n";
#endif // _PRINT_PROCESS_ 
            case 2  :
                result = "_" + zeabus::convert::integer::to_string( time->tm_mon , 2 , delim ) 
                        + result;
#ifdef _PRINT_PROCESS_
                std::cout   << "case 2 : " << result << "\n";
#endif // _PRINT_PROCESS_ 
            case 1  :
                result = zeabus::convert::integer::to_string( time->tm_year + 1900 , 4 , delim ) 
                        + result;
#ifdef _PRINT_PROCESS_
                std::cout   << "case 1 : " << result << "\n";
#endif // _PRINT_PROCESS_ 
                break;
        } // switch type_case
        return result;
    }

} // namespace zeabus
