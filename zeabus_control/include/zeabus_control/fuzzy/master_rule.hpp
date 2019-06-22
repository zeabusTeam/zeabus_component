// FILE			: master_rule.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, June 19 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README
//  This parameter for fuzzy_3_dimension of control package for set master rule

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <array>

namespace zeabus_control
{

namespace fuzzy
{

    const static std::array< std::array< std::array < short int , 7 > 7 > 7 > MASTER_RULE = 
    {
        // CASE fuzzy force = -3
        //  -3  -2  -1  +0  +1  +2  +3  = CASE error fuzzy
            +1, +3, +3, +4, +4, +4, +4, // CASE diff fuzzy = -3
            +0, +2, +3, +4, +4, +4, +4, // CASE diff fuzzy = -2
            -1, +0, +0, +4, +4, +4, +4, // CASE diff fuzzy = -1
            -1, -1, -1, +0, +3, +3, +4, // CASE diff fuzzy = -0
            -1, -4, -1, -4, +0, +0, +2, // CASE diff fuzzy = +1
            -4, -4, -4, -4, -2, -1, +0, // CASE diff fuzzy = +2
            -4, -4, -4, -4, -3, -2, -1, // CASE diff fuzzy = +3
        
        , // CASE fuzzy force = -2
        //  -3  -2  -1  +0  +1  +2  +3  = CASE error fuzzy
            +1, +3, +3, +4, +4, +4, +4, // CASE diff fuzzy = -3
            +0, +1, +2, +4, +4, +4, +4, // CASE diff fuzzy = -2
            -1, +0, +0, +3, +3, +3, +4, // CASE diff fuzzy = -1
            -1, -1, -1, +0, +2, +2, +3, // CASE diff fuzzy = -0
            -2, -2, -2, -1, +0, +0, +2, // CASE diff fuzzy = +1
            -3, -2, -2, -1, -1, -1, +0, // CASE diff fuzzy = +2
            -4, -4, -3, -3, -2, -2, -1, // CASE diff fuzzy = +3
        
        , // CASE fuzzy force = -1
        //  -3  -2  -1  +0  +1  +2  +3  = CASE error fuzzy
            +1, +2, +4, +4, +4, +4, +4, // CASE diff fuzzy = -3
            +0, +1, +2, +3, +3, +3, +4, // CASE diff fuzzy = -2
            -1, +0, +0, +2, +2, +3, +4, // CASE diff fuzzy = -1
            -2, -1, -1, +0, +2, +2, +3, // CASE diff fuzzy = -0
            -3, -2, -2, -1, +0, +0, +1, // CASE diff fuzzy = +1
            -4, -3, -3, -2, -2, -1, +0, // CASE diff fuzzy = +2
            -4, -4, -4, -4, -3, -2, +1, // CASE diff fuzzy = +3
        
        , // CASE fuzzy force = 0
        //  -3  -2  -1  +0  +1  +2  +3  = CASE error fuzzy
            +1, +2, +3, +4, +4, +4, +4, // CASE diff fuzzy = -3
            +0, +1, +3, +3, +3, +3, +4, // CASE diff fuzzy = -2
            -2, +0, +0, +2, +1, +3, +3, // CASE diff fuzzy = -1
            -3, -2, -1, +0, +1, +2, +3, // CASE diff fuzzy = -0
            -3, -3, --, -2, +0, +0, +2, // CASE diff fuzzy = +1
            -4, -3, -3, -3, -1, -1, +0, // CASE diff fuzzy = +2
            -4, -4, -4, -4, -2, -2, -1, // CASE diff fuzzy = +3
        
        , // CASE fuzzy force = +1
        //  -3  -2  -1  +0  +1  +2  +3  = CASE error fuzzy
            +1, +2, +3, +4, +4, +4, +4, // CASE diff fuzzy = -3
            +0, +1, +2, +3, +3, +3, +4, // CASE diff fuzzy = -2
            -1, +0, +0, +1, +2, +2, +3, // CASE diff fuzzy = -1
            -3, -2, -2, +0, +1, +1, +2, // CASE diff fuzzy = -0
            -4, -3, -2, -2, +0, +0, +1, // CASE diff fuzzy = +1
            -4, -3, -3, -3, -2, -1, +0, // CASE diff fuzzy = +2
            -4, -4, -4, -4, -4, -2, -1, // CASE diff fuzzy = +3
        
        , // CASE fuzzy force = +2
        //  -3  -2  -1  +0  +1  +2  +3  = CASE error fuzzy
            +1, +2, +2, +4, +4, +4, +4, // CASE diff fuzzy = -3
            +0, +1, +1, +1, +3, +3, +4, // CASE diff fuzzy = -2
            -2, +0, +0, +1, +2, +2, +2, // CASE diff fuzzy = -1
            -3, -2, -2, +0, +1, +1, +1, // CASE diff fuzzy = -0
            -4, -3, -3, -3, +0, +0, +1, // CASE diff fuzzy = +1
            -4, -4, -4, -4, -2, -1, +0, // CASE diff fuzzy = +2
            -4, -4, -4, -4, -3, -3, -1, // CASE diff fuzzy = +3
        
        , // CASE fuzzy force = +3
        //  -3  -2  -1  +0  +1  +2  +3  = CASE error fuzzy
            +1, +2, +3, +4, +4, +4, +4, // CASE diff fuzzy = -3
            +0, +1, +2, +4, +4, +4, +4, // CASE diff fuzzy = -2
            -2, +0, +0, +4, +1, +1, +1, // CASE diff fuzzy = -1
            -4, -3, -3, +0, +1, +1, +1, // CASE diff fuzzy = -0
            -4, -4, -4, -4, +0, +0, +1, // CASE diff fuzzy = +1
            -4, -4, -4, -4, -3, -2, +0, // CASE diff fuzzy = +2
            -4, -4, -4, -4, -3, -3, -1  // CASE diff fuzzy = +3
    };
    
} // namespace fuzzy

} // namespace zeabus_control
