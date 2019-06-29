// FILE         : interface.hpp
// AUTHOR       : Supasan Komonlit
// CREATE DATE  : 2019, MARCH, 25
// MAINTAINER   : K.Supasan

#include    <zeabus/sensor/IMU/packet.hpp>

#include    <zeabus/serial/synchronous_port.hpp>

#include    <zeabus/sensor/IMU/LORD_IMU_COMMUNICATION.hpp>

// Macro detail will print data for your purpose
//      _PRINT_DATA_CONNECTION_ for print packet command
//      _CHECK_MEMORY_ for print detail memory about packet <vector in c++ language>
//      _ERROR_TYPE_ for data information if return false why we return that

#ifndef _ZEABUS_SENSOR_IMU_INTERFACE_HPP__
#define _ZEABUS_SENSOR_IMU_INTERFACE_HPP__

namespace zeabus
{

namespace sensor
{

namespace IMU
{

    class Interface : public Packet , public zeabus::serial::SynchronousPort
    {
        public:
            Interface( std::string port_name = "" , unsigned int size = 100 );

            bool set_idle(); // this command will stop stream and set imu ready for setting
            bool ping(); // ping command
            bool resume(); // this command you to continue stream
                    // WARNING this command didn't set un/enable stream data

            // please send in mode or Rate Decimation
            // calculate by Rate Decimation = IMU Base Rate / Desired Data Rate
            void set_IMU_rate( int rate ); 

            bool set_IMU_message_format( unsigned char first_type , unsigned char second_type
                    , unsigned char thrid_type );

            // IMU have 2 type of data stream this function stream only IMU type
            bool enable_IMU_data_stream();

            bool read_stream(); // this command we will read data for collecting stream DATA
                    // In mode IMU data. WARNING We think this function canread

            // This command to set new value from capture gyro bias
            // Can study on page 69 for setup data and page 70 for capture 
            bool auto_set_gyro_bias();

        protected:
            // we split to 2 variable because our individual member is unsigned char
            unsigned char front_rate;
            unsigned char back_rate;

            void init_header();

            bool read_reply( unsigned char descriptor_byte , unsigned int max_round = 1 );

        private:
            std::vector<unsigned char> reader_buffer;
    };

}

}

}

#endif // _ZEABUS_SENSOR_IMU_INTERFACE_HPP__
