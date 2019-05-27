// FILE         : synchronous_port.cpp
// AUTHOR       : Supasan Komonlit
// CREATE DATE  : 2019, MARCH, 22

#include    <zeabus/serial/synchronous_port.hpp>

// MACRO DETAIL
// _SHOW_INDIVIDUAL_CHAR_ : this for read_data( std::string* ) this show data haved read
// _SHOW_RESULT_MESSAGE_  : this for read_data( std::string* ) this show result of message

// MACRO SET
//#define _SHOW_INDIVIDUAL_CHAR_
//#define _SHOW_RESULT_MESSAGE_

// MACRO CONDITION
#ifdef _SHOW_INDIVIDUAL_CHAR_
    #define _SHOW_RESULT_MESSAGE_
#endif

namespace zeabus
{

namespace serial
{

    SynchronousPort::SynchronousPort( std::string port_name ) : SerialPort( port_name ){}

    // About size of vector to use buffer for read and write serial if your vector size
    // don't enough for collect or write data that will make you can do size = current_size

    unsigned int SynchronousPort::read_data( std::vector<unsigned char>* buffer 
            , unsigned int size )
    {
        if( (*buffer).size() < size )
        {
            (*buffer).resize( size ); // must ensure your buffer have size can collect data
        }
        unsigned int size_data;
        do
        {   
            size_data = boost::asio::read( this->io_port , boost::asio::buffer( *buffer , size )
                    , this->error_code );
            if( this->error_code == _boost_errc::resource_unavailable_try_again 
                    || this->error_code == _boost_errc::interrupted )
            {
                std::cout   << "Port unavailable for reading will try again\n";
            }
            else if( this->error_code == _boost_errc::success )
            {
                break;
            }
            else
            {
                std::cout   << "Failure to reading error_code is " 
                            << (this->error_code).value() << "\n";
                size_data = 0;
            }
        }while( true );
        return size_data;
    } // function read_data

    unsigned int SynchronousPort::write_data( std::vector<unsigned char>* buffer 
            , unsigned int size )
    {
        unsigned int size_data = boost::asio::write( this->io_port 
                , boost::asio::buffer( *buffer , size ) , this->error_code );
        if( this->error_code != _boost_errc::success )
        {
            std::cout   << "Failure for writing data to Synchronous port error_code is "
                        << (this->error_code).value() << "\n" ;
        }
        return size_data;
    } // function write_data

    // next will part to read and wirte data by argument std::string 
    unsigned int SynchronousPort::read_data( std::string* message )
    {
        *message = ""; // reset message to empty string
        unsigned int count = 0 ; // for count number of byte we can read
        char temporary;
        bool continue_read = true;
        while( continue_read )
        {
            count += boost::asio::read( this->io_port , boost::asio::buffer( &temporary , 1 ) );
            switch( temporary )
            {
            case '\r' :
#ifdef _SHOW_INDIVIDUAL_CHAR_
                std::cout << "<read_string> start cursor\n";
#endif // _SHOW_INDIVIDUAL_CHAR_
                continue;
            case '\n' :
#ifdef _SHOW_INDIVIDUAL_CHAR_
                std::cout << "<read_string> new line\n";
#endif // _SHOW_INDIVIDUAL_CHAR_
                continue_read = false;
            case '\0' :
#ifdef _SHOW_INDIVIDUAL_CHAR_
                std::cout << "<read_string> end of string\n";
#endif // _SHOW_INDIVIDUAL_CHAR_
                continue;
            default :
                *message += temporary;
                count += 1;
#ifdef _SHOW_INDIVIDUAL_CHAR_
                std::cout << "<read_string> data is \"" << temporary << "\"\n"; 
#endif // _SHOW_INDIVIDUAL_CHAR_
            }
        } // loop while read string
#ifdef _SHOW_RESULT_MESSAGE_
        std::cout << "<read_string> Message data is " << *message << "\n";
#endif // _SHOW_INDIVIDUAL_CHAR_
        return count;
    } // function read_string

    unsigned int SynchronousPort::write_data( std::string* message )
    {
        return boost::asio::write( this->io_port , boost::asio::buffer( *message) 
                , boost::asio::transfer_all() , this->error_code );
    } // function write_string

} // namespace serial

} // namespace zeabus

