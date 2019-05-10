// FILE			: to_string.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 10 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

namespace zeabus
{

namespace convert
{

    template< typename input_type >
    std::string to_string( input_type input_data )
    {
        std::basic_stringstream< char > temp_stringstream;
        temp_stringstream   << input_data; // this inherit from basic_ostream
        return temp_stringstream.str(); 
    } // to_string function

} // namespace convert

} // namespace zeabus
