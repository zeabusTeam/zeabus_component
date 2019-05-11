// FILE			: time.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 11 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET
//#define _PRINT_PROCESS_

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
        std::string result = "";
        std::stringstream temporary;
        switch( type_case )
        {
            case 6  :
                temporary.clear();
                temporary << std::setw(2) << std::setfill('0') << time->tm_sec;
                result = "_" + temporary.str() + result;
#ifdef _PRINT_PROCESS_
                std::cout   << "case 6 : " << result << "\n";
#endif 
            case 5  :
                temporary.clear();
                temporary << std::setw(2) << std::setfill('0') << time->tm_min;
                result = "_" + temporary.str() + result;
#ifdef _PRINT_PROCESS_
                std::cout   << "case 5 : " << result << "\n";
#endif 
            case 4  :
                temporary.clear();
                temporary << std::setw(2) << std::setfill('0') << time->tm_hour;
                result = "_" + temporary.str() + result;
#ifdef _PRINT_PROCESS_
                std::cout   << "case 4 : " << result << "\n";
#endif 
            case 3  :
                temporary.clear();
                temporary << std::setw(2) << std::setfill('0') << time->tm_mday;
                result = "_" + temporary.str() + result;
#ifdef _PRINT_PROCESS_
                std::cout   << "case 3 : " << result << "\n";
#endif 
            case 2  :
                temporary.clear();
                temporary << std::setw(2) << std::setfill('0') << time->tm_mon;
                result = "_" + temporary.str() + result;
#ifdef _PRINT_PROCESS_
                std::cout   << "case 2 : " << result << "\n";
#endif 
            case 1  :
                temporary.clear();
                temporary << std::setw(4) << std::setfill('0') << time->tm_year;
                result = temporary.str() + result;
#ifdef _PRINT_PROCESS_
                std::cout   << "case 1 : " << result << "\n";
#endif 
                break;
        } // switch type_case
        return result;
    }

} // namespace zeabus
