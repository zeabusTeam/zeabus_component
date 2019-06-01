// FILE			: read_data.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 31 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

std::string read_bool( bool data )
{
    std::string result;
    if( data )
    {
        result = "True";
    }
    else
    {
        result = "False";
    }
    return result;
}

int read_bit_value( unsigned char status )
{
    int result = 0;
    if( (status & 0b001) == 1 ) 
    {
        result += 1;
    }
    if( (status & 0b010) == 1 ) 
    {
        result += 2;
    }
    if( (status & 0b100) == 1 ) 
    {
        result += 4;
    }
    return result;
}
