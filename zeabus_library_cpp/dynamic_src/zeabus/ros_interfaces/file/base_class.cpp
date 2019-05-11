// FILE			: base_class.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 11 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

namespace zeabus
{

namespace ros_interfaces
{

namespace file
{

    BaseClass::BaseClass( std::string full_path ) : PathFile( full_path )
    {
        ;
    }

    void BaseClass::close()
    {
        if( this->is_open() )
        {
            (this->stream_file).close();
        }
    }

    bool BaseClass::open( std::string full_path )
    {
        (this->stream_file).open( full_path );
        return (this->stream_file).is_open();
    }

    bool BaseClass::open( )
    {
        bool result = this->updated_path();
        if( result )
        {
            (this->stream_file).open( this->full_path );
        }
        return result;
    }

    bool BaseClass::is_open()
    {
        return (this->stream_file).is_open();
    }

    template< typename type_output >
    void write( type_output* output )
    {
        this->stream_file << output;
    }

    template< typename type_input >
    void read( type_output* input )
    {
        this->stream_file >> *input;
    }

} // namespace file

} // namespace ros_interfaces

} // namespace zeabus
