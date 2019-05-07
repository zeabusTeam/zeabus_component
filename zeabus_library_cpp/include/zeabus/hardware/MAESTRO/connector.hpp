//  FILE        : connector.hpp
//  AUTHOR      : Supasan Komonlit
//  CREATE DATE : 2019, APRIL 29
//  REFERENCE   : This code will write follow old pattern pololu_packet.h but split to two part
//      FILE        : pololu_packet.h
//      AUTHOR      : Mahisorn
//      CREATED ON  : Nov 28, 2014
//  MAINTAINTER : Supasan Komonlit

#include    <iostream> // standard library of c++ language

#include    <zeabus/serial/synchronous_port.hpp>

#include    <zeabus/hardware/MAESTRO/packet.hpp>

#include    <zeabus/hardware/MAESTRO/protocol.hpp>

// This file will prepare packet and write packet
// reference https://www.pololu.com/docs/0J40/5.e

// MACRO_DETAIL
// _PRINT_INPUT_    : use to print std::vector
// _PRINT_BEFORE_WRITE_

//#define _PRINT_INPUT_
//#define _PRINT_BEFORE_WRITE_

#ifndef _ZEABUS_HARDWARE_MAESTRO_CONNECTOR_HPP__
#define _ZEABUS_HARDWARE_MAESTRO_CONNECTOR_HPP__

namespace zeabus
{

namespace hardware
{

namespace MAESTRO
{

namespace POLOLU
{

    class Connector : public Packet , public zeabus::serial::SynchronousPort 
    {
        public:
            // In case data for all
            Connector( std::string port_name , unsigned char init_byte 
                    = zeabus::hardware::MAESTRO::protocol::POLOLU::BASE_PROTOCOL 
                    , unsigned char device_number = 0x0C , unsigned int reserve_size = 100 );

            bool set_multiple_targets( std::vector<unsigned short int >* target_bits 
                    , unsigned char first_channel = 0 );

    }; // class connector

} // namespace POLOLU

} // namespace MAESTRO

} // namespace hardware

} // namespace zeabus

#endif // _ZEABUS_SENSOR_MAESTRO_CONNECTOR_HPP__
