// FILE         : plololu_packet.cpp
// AUTHOR       : Supasan Komonlit
// CREATE ON    : 2019, APRIL 11

#include    <zeabus/hardware/MAESTRO/packet.hpp>

// This file will source code only packet for Pololu Protocol

namespace zeabus
{

namespace hardware
{

namespace MAESTRO
{

namespace POLOLU
{

    Packet::Packet( unsigned char init_byte , unsigned char device_number 
            , unsigned int reserve_size ) : BasePacket( reserve_size )
    {
        this->set_init_byte( init_byte );
        this->set_device_number( device_number );
    } // init function of Packet class 

    void Packet::set_device_number( unsigned char device_number )
    {
        this->device_number = device_number;
    } // function set_device_number

    void Packet::set_init_byte( unsigned char init_byte )
    {
        this->init_byte = init_byte;
    } // function set_init_byte

    void Packet::init_header( unsigned char command )
    {
        this->clear_member();
        zeabus::variadic::push_data( &(this->data) , this->init_byte , this->device_number 
                , command ); // push 3 byte set header device_number and command
    } // function init_header

    void Packet::push_vector_2_bytes( std::vector< unsigned short int >* data )
    {
        unsigned char low_bits;
        unsigned char high_bits;
        unsigned short int temp_target;
        for( std::vector< unsigned short int>::iterator point = data->begin() ; 
                point != data->end() ; point++ )
        {
            temp_target = (*point) << 2;
#ifdef _PRINT_CONVERT_
            std::cout   << "Original data " << (temp_target) << std::hex; 
#endif // _PRINT_CONVERT_
//            low_bits = ( *point ) & 0x7F; // we want data 7 bits in 1 bytes
//            high_bits = ( ( *point ) >> 7 ) & 0x7F; // shift right 7 bits make low_bits out
            low_bits = ( temp_target ) & 0x7F; // we want data 7 bits in 1 bytes
            high_bits = ( ( temp_target ) >> 7 ) & 0x7F; // shift right 7 bits make low_bits out
                    // and use only 7 bits in 1 bytes
#ifdef _PRINT_CONVERT_
            std::cout   << " <----> New data " << ( 0xFF & high_bits) << " : " 
                        << ( 0xFF & low_bits ) << std::dec << "\n"
                        << "----------------------------------------------------------------\n";
#endif // _PRINT_CONVERT_
            zeabus::variadic::push_data( &(this->data) , low_bits , high_bits);
        } // end for loop 
    } // function push_vector_2_bytes

} // namespace POLOLU

} // namespace MAESTRO

} // namespace hardware

} // namespace zeabus
