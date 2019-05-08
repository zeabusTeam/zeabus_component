// FILE         : swap.cpp
// AUTHOR       : K.Supasan
// CREATE ON    : 2019, APRIL 8
// MAINTAINER   : K.Supasan

namespace zeabus
{

    template< typename type_value >
    void swap( type_value* source , type_value* destination )
    {
        type_value temporary = *source;
        *source = *destination;
        *destination = temporary;
    }

} // namespace zeabus
