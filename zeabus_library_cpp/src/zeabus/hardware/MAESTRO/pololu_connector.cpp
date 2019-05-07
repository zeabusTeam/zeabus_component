// FILE         : pololu_connector.cpp
// AUTHOR       : Supasan Komonlit
// CREATE ON    : 2019, APRIL 11

#include    <zeabus/hardware/MAESTRO/connector.hpp>

namespace zeabus
{

namespace hardware
{

namespace MAESTRO
{

namespace POLOLU
{

    Connector::Connector( std::string port_name , unsigned char init_byte 
            , unsigned char device_number , unsigned int reserve_size ) : Packet( init_byte 
                    , device_number , reserve_size ) , SynchronousPort( port_name )
    {
        ; // for case you set start data all
    } // init function case 4

    bool Connector::set_multiple_targets( std::vector< unsigned short int >* target_bits
            , unsigned char first_channel )
    {
        bool result = false;
        this->init_header( 
                zeabus::hardware::MAESTRO::protocol::POLOLU::COMMAND_SET_MULTIPLE_TARGETS );
        zeabus::variadic::push_data( &(this->data) , target_bits->size() , first_channel );
#ifdef _PRINT_INPUT_
        std::cout   << "================ SET MULTIPLE TARGETS ======================\n";
        for( unsigned int run = 0 ; run < target_bits->size() ; run++ )
        {
            std::cout   << "  " << (*target_bits)[ run ];
        }
        std::cout   << "\n============================================================\n";
#endif // _PRINT_INPUT_
        this->push_vector_2_bytes( target_bits );
#ifdef _PRINT_BEFORE_WRITE_
        this->print_data( "set multiple targets");
#endif // _PRINT_BEFORE_WRITER_
        unsigned int num_check = this->write_data( &(this->data) , (this->data).size() );
        if( num_check == (this->data).size() )
        {
            result = true;
        } // if condition result = true
        return result;
    } // function set_multiple_targets

} // namespace POLOLU

} // namespace MAESTRO

} // namespace hardware

} // namespace zeabus
