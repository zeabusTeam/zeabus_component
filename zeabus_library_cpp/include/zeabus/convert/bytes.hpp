// FILE         : bytes.hpp
// AUTHOR       : Supasan Komonlit
// CREATE DATE  : 2019, APRIL 24
// MAINTAINER   : Supasan Komonlit

// Include standard library of cpp
#include    <iostream>

// import for function memcpy to convert unsigned char * 4 to float
#include    <cstring> 

// import type of raw data to use 
#include    <vector>

// This file create for convert data bytes to variable type int / float or double up to you 
// But ensure you have to study about size of type varaible and your data that are correct
// for size of type varaible we reference from http://www.cplusplus.com/doc/tutorial/variables/

// MACRO DETAIL PART


#ifndef _ZEABUS_CONVERT_BYTES_HPP__
#define _ZEABUS_CONVERT_BYTES_HPP__

namespace zeabus
{

namespace convert
{

namespace bytes
{

    // we design to send by pointer because that is help and quickly to pass value
    // we think that we help
    
    // float have design to have size is 4 bytes that make me use for imu packet
    // because 1 variable have 4 bytes 
    bool vector_to_float( std::vector<unsigned char>* set_data 
            , float* answer , unsigned int offset = 0 );

    // double have size is 8 bytes
    bool vector_to_double( std::vector<unsigned char>* set_data
            , double* answer , unsigned int offset = 0 );

} // namespace convert

} // namespace convert

} // namespace zeabus

#endif // _ZEABUS_CONVERT_BYTES_HPP__
