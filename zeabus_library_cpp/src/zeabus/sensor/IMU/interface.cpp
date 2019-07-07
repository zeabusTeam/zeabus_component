// FILE         : interface.cpp
// AUTHOR       : Supasan Komonlit
// CREATE DATE  : 2019, MARCH 25

#include    <zeabus/sensor/IMU/interface.hpp>

// All code have reference from data-sheet of 3dm-gx5-45_data-commmunication_protocol.pdf

namespace zeabus
{

namespace sensor
{

namespace IMU
{

    Interface::Interface( std::string port_name , unsigned int size ) : 
            SynchronousPort( port_name ) , Packet( size )
    {
        (this->reader_buffer).reserve( (unsigned int) 100 );
    }

    bool Interface::set_idle()
    {
        bool result = false;
        unsigned int num_check;
        this->init_header(); // Init header file for buffer
        variadic::push_data( &(this->data) , LORD_MICROSTRAIN::COMMAND::BASE::DESCRIPTOR , 0x02 
                , 0x02 , LORD_MICROSTRAIN::COMMAND::BASE::IDLE );
        this->add_check_sum();
        num_check = this->write_data( &(this->data) , (this->data).size() );
        if( num_check != (this->data).size() )
        {
            ; // In case can't write 
        }
        else
        {
            if( this->read_reply( LORD_MICROSTRAIN::COMMAND::BASE::DESCRIPTOR ) )
            {
                if( this->check_sum() )
                {
                    // This is one case to can make result is true
                    result = ( *( (this->data).end() - 3 ) == 0x00 );
                }
            }
        }
        return result;
    }

    bool Interface::ping()
    {
        bool result = false;
        unsigned int num_check;
        this->init_header();
        variadic::push_data( &(this->data) , LORD_MICROSTRAIN::COMMAND::BASE::DESCRIPTOR 
                , 0x02 , 0x02 , LORD_MICROSTRAIN::COMMAND::BASE::PING );
        this->add_check_sum();
        num_check = this->write_data( &(this->data) , (this->data).size() );
        if( num_check != (this->data).size() )
        {
            ; // In case can't write data equal size of packet
        }
        else
        {
            if( this->read_reply( LORD_MICROSTRAIN::COMMAND::BASE::DESCRIPTOR ) )
            {
                if( this->check_sum() )
                {
                    // This is one case to return true we check descriptor checksum and ACK
                    result = ( *( ( this->data).end() - 3 ) == 0x00 ); 
                }
            }
        }
        return result;
    }

    void Interface::set_IMU_rate( int rate )
    {
        this->front_rate = (unsigned char ) ( ( (rate) & 0xff00 ) >> 8 );
        this->back_rate = (unsigned char) ( rate & 0xff );
    }

    bool Interface::set_IMU_message_format( unsigned char first_type , unsigned char second_type
            , unsigned char third_type ) // This function page 51
    {
        bool result = false;
        unsigned int num_check;
        // Part prepare data for writing
        this->init_header();
        variadic::push_data( &(this->data) , LORD_MICROSTRAIN::COMMAND::SENSOR::DESCRIPTOR , 0x0d
                , 0x0d , LORD_MICROSTRAIN::COMMAND::SENSOR::IMU_MESSAGE_FORMAT , 0x01 
                , 0x03 , first_type , this->front_rate , this->back_rate 
                , second_type , this->front_rate , this->back_rate 
                , third_type , this->front_rate , this->back_rate );
        this->add_check_sum();
        num_check = this->write_data( &(this->data) , (this->data).size() );
        if( num_check != (this->data).size() )
        {
            ; // in case can write data equal lenght of packet
        }
        else
        {
            if( this->read_reply( LORD_MICROSTRAIN::COMMAND::SENSOR::DESCRIPTOR ) )
            {
                if( this->check_sum() )
                {
                    result = ( *( (this->data).end() - 3 ) == 0x00 );
                }
            }
        }
        return result;
    }

    bool Interface::enable_IMU_data_stream()
    {
        bool result = false;
        unsigned int num_check;
        this->init_header();
        variadic::push_data( &(this->data) , LORD_MICROSTRAIN::COMMAND::SENSOR::DESCRIPTOR , 0x0a
                , 0x05 , LORD_MICROSTRAIN::COMMAND::SENSOR::CONTINUOUS, 0x01 , 0x01 , 0x01 
                , 0x05 , LORD_MICROSTRAIN::COMMAND::SENSOR::CONTINUOUS , 0x01 , 0x03 , 0x00 );
        this->add_check_sum();
        num_check = this->write_data( &(this->data) , (this->data).size());
        if( num_check != (this->data).size() )
        {
            ; // will return false for can write == want write
        }
        else
        {
            if( this->read_reply( LORD_MICROSTRAIN::COMMAND::SENSOR::DESCRIPTOR ) )
            {
                if( this->check_sum() )
                {
                    result = ( *( (this->data).end() - 3 ) == 0x00 );  
                }
            }
        }
        return result;
    }

    bool Interface::auto_set_gyro_bias( bool check_value )
    {
        std::cout   << "Start to capture gyro bias\n";
        bool result = false;
        unsigned int num_check;
        unsigned int count = 0;
        unsigned int limit_round = 50;
        this->init_header();
        variadic::push_data( &(this->data), LORD_MICROSTRAIN::COMMAND::SENSOR::DESCRIPTOR , 0x04
                , 0x04 , LORD_MICROSTRAIN::COMMAND::SENSOR::CAPTURE_GYRO_BIAS, 0x0b , 0xb8 );
        this->add_check_sum();
        num_check = this->write_data( &( this->data ) , (this->data).size() );
        if( num_check != ( this->data).size() )
        {
            std::cout   << "IMU Failure to write command capture gyro bias\n";
            count = limit_round + 1;
            ; // will return false for can write == want write
        }
        else
        {
            std::cout   << "Success to write_data";
        }
    
        std::vector< unsigned char > temp_vector;
        while( ( count <= limit_round ) && (! result) )
        {
            count++;
            if( this->read_reply( LORD_MICROSTRAIN::COMMAND::SENSOR::DESCRIPTOR , 5) )
            {
                if( (  (this->data)[6] ) != 0x39 )
                {
                    std::cout   << "Wrong command" << "\n"; 
                    continue;
                }
                else
                {
                    std::cout   << "Correct command\n";
                }
                if( this->check_sum() )
                {
                    this->print_data( " data is " );
                    result = ( *( (this->data).end() - 17 ) == 0x00 );
                    std::cout   << "data on temp is"; 
                    for( unsigned int run = 10 ; run < 22 ; run++ )
                    {
                        temp_vector.push_back( (this->data).at(run) );
                        printf( " %x" , temp_vector[ run - 10 ] );
                    }
                    std::cout   << "\n";
                }
            }
            std::cout   << "Round " << count << "\n";
            printf( "data point to check is %x\n" , *( (this->data).end() - 17 ) );
        }
        std::cout   << "Finish process get data\n";
        // Next process will use only case you have success capture gyro bias
        if( result )
        {
            float first_gyro_bias;
            float second_gyro_bias;
            float third_gyro_bias;
            char save_value = 'y';
            zeabus::convert::bytes::vector_to_float( &temp_vector , &first_gyro_bias , 0 );
            std::cout   << "Convert first data\n";  
            zeabus::convert::bytes::vector_to_float( &temp_vector , &second_gyro_bias , 4 ); 
            std::cout   << "Convert Second data\n";  
            zeabus::convert::bytes::vector_to_float( &temp_vector , &third_gyro_bias , 8 ); 
            std::cout   << "Vector of capture bias value is :\n\t"
                        << first_gyro_bias << "\n\t"
                        << second_gyro_bias << "\n\t"
                        << third_gyro_bias << "\n";
            if( check_value )
            {
                std::cout   << "Please enter char y to save start value : ";
                std::cin    >> save_value;  
            } //  normal situation we have want you to save data
            if( save_value != 'y' )
            {
                std::cout   << "You don't want save value out of function\n";
            }
            else
            {
                result = false;
                // init_header for create vector save value 
                std::cout   << "Start to write for get back current value\n";
                this->init_header();
                variadic::push_data( &(this->data) 
                    , LORD_MICROSTRAIN::COMMAND::SENSOR::DESCRIPTOR , 0x0f , 0x0f 
                    , LORD_MICROSTRAIN::COMMAND::SENSOR::GYRO_BIAS, 0x01 );
                for( unsigned int run = 0 ; run < 12 ; run++ )
                {
                    (this->data).push_back( temp_vector[ run ] );
                }
                this->add_check_sum();
                this->print_data( "Before write " );
                num_check = this->write_data( &( this->data) , (this->data).size() );
                std::cout   << "Finish write\n";
                if( num_check != (this->data).size() )
                {
                    std::cout   << "Failure to write get current value gyro bias\n";
                }
                else
                {
                    count = 0;
                    while( count < limit_round )
                    {
                        count++;
                        if( this->read_reply( LORD_MICROSTRAIN::COMMAND::SENSOR::DESCRIPTOR ) )
                        {
                            this->print_data( "Reply " );
                            if( (  (this->data)[6] ) != 0x38 )
                            {
                                std::cout   << "Wrong command" << "\n"; 
                                continue;
                            }
                            else
                            {
                                std::cout   << "Correct command\n";
                            }
                            if( this->check_sum() )
                            {    
                                result = ( *( (this->data).end() - 3 ) == 0x00 );
                                if( result == true )
                                {
                                    break;
                                }
                            }
                        }
                    }
                }
            }
            if( result == false )
            {
                std::cout   << "Failure to read data back\n";
            }
            else if( save_value == 'y' )
            {
                this->init_header();
                variadic::push_data( &( this->data )
                    , LORD_MICROSTRAIN::COMMAND::SENSOR::DESCRIPTOR , 0x03 , 0x03 
                    , LORD_MICROSTRAIN::COMMAND::SENSOR::GYRO_BIAS , 0x02 );
                this->add_check_sum();
                num_check = this->write_data( &(this->data) , (this->data).size() );

                if( num_check != (this->data).size() )
                {
                    std::cout   << "Failure to write new capture gyro bias\n";
                }
                else if( this->read_reply( LORD_MICROSTRAIN::COMMAND::SENSOR::DESCRIPTOR ,100 ) )
                {
                    if( this->check_sum() )
                    {
                        this->print_data( "Result feedback : ");
                        result = ( *( (this->data).end() - 17 ) == 0x00 );
                        if( result )
                        {
                            std::vector< unsigned char > temp_current_vector;
                            for( unsigned int run = 10 ; run < 21 ; run ++ )
                            {
                                temp_current_vector.push_back( (this->data).at(run) );
                            }
                            float first_current;
                            float second_current;
                            float third_current;
                            zeabus::convert::bytes::vector_to_float( &temp_current_vector 
                                , &first_current , 0 ); 
                            zeabus::convert::bytes::vector_to_float( &temp_current_vector 
                                , &second_current , 4 ); 
                            zeabus::convert::bytes::vector_to_float( &temp_current_vector 
                                , &third_current , 8 ); 
                            std::cout   << "Vector of current bias value is :\n\t"
                                        << first_current << "\n\t"
                                        << second_current << "\n\t"
                                        << third_current << "\n";
                            std::cout   << "Vector of capture bias value is :\n\t"
                                        << first_gyro_bias << "\n\t"
                                        << second_gyro_bias << "\n\t"
                                        << third_gyro_bias << "\n";
                        }
                        else
                        {
                            std::cout   << "Failure to read back current setting\n";
                        }
                    }
                }
            } // condition want to cheack value
            else
            {
                ;
            }
        } // condition to about read back current setting from new gyro bias value
        if( result )
        {
            char save_value = 'y';
            if( check_value )
            {
                std::cout   << "Please, enter y is you want to save startup : ";
                std::cin    >> save_value;
            }
            if( save_value != 'y' ) 
            {
                std::cout   << "You did't want save to start up value\n";
            }
            else
            {
                this->init_header();
                variadic::push_data( &( this->data )
                    , LORD_MICROSTRAIN::COMMAND::SENSOR::DESCRIPTOR , 0x03 , 0x03 
                    , LORD_MICROSTRAIN::COMMAND::SENSOR::GYRO_BIAS , 0x03 );
                this->add_check_sum();
                if( num_check != ( this->data).size() )
                {
                    result = false;
                    ; // will return false for can write != want write
                }
                else
                {
                    if( this->read_reply( LORD_MICROSTRAIN::COMMAND::SENSOR::DESCRIPTOR ) )
                    {
                        if( this->check_sum() )
                        {
                            result = ( *( (this->data).end() - 3 ) == 0x00 );
                            if( result )
                            {
                                std::cout   << "Success save startup setting\n";
                            }
                        }
                    }
                }
            }
        } // condition about save to startup setting
        this->resume();
        return result;
    }


    bool Interface::resume()
    {
        bool result = false;
        unsigned int num_check;
        this->init_header();
        variadic::push_data( &(this->data) , LORD_MICROSTRAIN::COMMAND::BASE::DESCRIPTOR 
                , 0x02 , 0x02 , LORD_MICROSTRAIN::COMMAND::BASE::RESUME );
        this->add_check_sum();
        num_check = this->write_data( &(this->data) , (this->data).size() );
        if( num_check != ( this->data).size() )
        {
            ; // will return false for can write != want write
        }
        else
        {
            if( this->read_reply( LORD_MICROSTRAIN::COMMAND::SENSOR::DESCRIPTOR ) )
            {
                if( this->check_sum() )
                {
                    result = ( *( (this->data).end() - 3 ) == 0x00 );
                }
            }
        }
        return result;
    }

    void Interface::init_header()
    {
        this->clear_member();
        variadic::push_data( &(this->data) , 0x75 , 0x65 );
    }

    bool Interface::read_reply( unsigned char descriptor_byte ,unsigned int max_round )
    {
        bool result = false;
        for( unsigned int round = 0 ; ( round < max_round ) ; round++ )
        {
            this->reader_buffer.resize( 1 );
            for( unsigned int individual = 0 ;  individual < 5 ; individual++ )
            {
                (void)this->read_data( &(this->reader_buffer) , 1 );
                if( (this->reader_buffer)[0] == 'u' )
                {
                    individual = 4;
                    result = true;
                } 
                else if( individual == 4 )
                {
                    result = false;
                }
                else
                {
                    ; // for MISRA C++ 
                }
            }
            if( result )
            {
                result = false;
                (void)this->read_data( &(this->reader_buffer) , 1 );
                if( (this->reader_buffer)[0] != 'e' )
                {
                    continue;
                }
                this->init_header();  
                (this->reader_buffer).resize( 2 );
                (void)(this->read_data( &(this->reader_buffer) , 2 ));
                this->push_vector( &(this->reader_buffer) );
                (this->reader_buffer).resize( (this->data)[3] );
                (void)(this->read_data( &(this->reader_buffer) , (this->reader_buffer).size() ));
                this->push_vector( &(this->reader_buffer) );
                (this->reader_buffer).resize( 2 );
                (void)(this->read_data( &(this->reader_buffer) , 2 ));
                this->push_vector( &(this->reader_buffer) );
                if( (this->data)[2] == descriptor_byte )
                {
                    result = true;
                    break;
                } 
            }
        }
        return result;
    }

    bool Interface::read_stream()
    {
        bool result = this->read_reply( 0x80 );
        if( result )
        {
            result = this->check_sum();
        }
        return result; 
    }

}

}

}
