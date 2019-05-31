// FILE         : decode_string.cpp
// AUTHOR       : Supasan Komonlit
// CREATE ON    : 2019, APRIL 09
// MAINTAINER   : Supasan Komonlit

#include    <zeabus/sensor/DVL/decode_string.hpp>

//#define _SHOW_DATA_

namespace zeabus
{

namespace sensor
{

namespace DVL
{

    // Pattern :BS,sXXXXX,sYYYYY,sZZZZZ,S
    void PD6_code_BS( std::string* message, int* vel_x, int* vel_y, int* vel_z, char* status )
    {
        std::string temp_message = "";    
        for( int count = 0 ; count < 6 ; count++ )
        {
            temp_message.push_back( (*message)[ 4 + count ]);
        } // loop for... vel_x
#ifdef _SHOW_DATA_
        std::cout   << "For vel_x data is " << temp_message;
#endif
        (void)zeabus::convert::string::to_integer( &temp_message , vel_x );

        temp_message = "";
        for( int count = 0 ; count < 6 ; count++ )
        {
            temp_message.push_back( (*message)[ 11 + count ]);
        } // loop for... vel_y
#ifdef _SHOW_DATA_
        std::cout   << "\nFor vel_y data is " << temp_message;
#endif
        (void)zeabus::convert::string::to_integer( &temp_message , vel_y );

        temp_message = "";
        for( int count = 0 ; count < 6 ; count++ )
        {
            temp_message.push_back( (*message)[ 18 + count ] );
        } // loop for... vel_z
#ifdef _SHOW_DATA_
        std::cout   << "\nFor vel_z data is " << temp_message << std::endl;
#endif
        (void)zeabus::convert::string::to_integer( &temp_message , vel_z );

        *status = (*message)[ 25 ]; 
    } // function PD6_code_BS

    // Pattern :BI,sXXXXX,sYYYYY,sZZZZZ,sEEEEE,S
    void PD6_code_BI( std::string* message, int* vel_x, int* vel_y, int* vel_z
            , int* vel_error , char* status )
    {
        std::string temp_message = "";    
        for( int count = 0 ; count < 6 ; count++ )
        {
            temp_message.push_back( (*message)[ 4 + count ] );
        } // loop for... vel_x
        (void)zeabus::convert::string::to_integer( &temp_message , vel_x );

        temp_message = "";
        for( int count = 0 ; count < 6 ; count++ )
        {
            temp_message.push_back( (*message)[ 11 + count ] );
        } // loop for... vel_x
        (void)zeabus::convert::string::to_integer( &temp_message , vel_y );

        temp_message = "";
        for( int count = 0 ; count < 6 ; count++ )
        {
            temp_message.push_back( (*message)[ 18 + count ] );
        } // loop for... vel_x
        (void)zeabus::convert::string::to_integer( &temp_message , vel_z );

        temp_message = "";
        for( int count = 0 ; count < 6 ; count++ )
        {
            temp_message.push_back( (*message)[ 25 + count ] );
        } // loop for... vel_x
        (void)zeabus::convert::string::to_integer( &temp_message , vel_error );

        *status = (*message)[ 32 ]; 
    } // function PD6_code_BI
    

} // namespace DVL

} // namespace sensor

} // namespace zeabus
