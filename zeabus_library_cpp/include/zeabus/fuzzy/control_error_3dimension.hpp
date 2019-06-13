// FILE			: control_error_3dimension.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, June 12 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README
//  This code is for zeabus control by fuzzy algorithm version 2.
//  We will use ros publisher to generate topic to publish data for you can anlysis data
//  We will auto publish data to topic have send on constuctor object
//  ------------------------- PROCESS DIAGRAM OF ACTIVE CALCULATE -----------------------------
//  intput -> push_error -> push_diff -> push_force -> fuzzy_rule -> defuzzification -> output
//  -------------------------------------------------------------------------------------------
//  output in this only relative value if you want to look about real result to use please
//  look on variable result that is value after sum all data

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <cmath>

#include    <array>

#include    <iostream>

#include    <ros/console.h>

#include    <ros/publisher.h>

#include    <ros/node_handle.h>

#include    <zeabus/escape_code.hpp>

#include    <zeabus_utility/ControlFuzzy.h>

#ifndef _ZEABUS_FUZZY_CONTROL_ERROR_3DIMENSION_HPP__
#define _ZEABUS_FUZZY_CONTROL_ERROR_3DIMENSION_HPP__

namespace zeabus
{

namespace fuzzy
{

    class ControlError3Dimension
    {

        public:
            // first argument std::string topic_name will use to make topic publish data
            ControlError3Dimension( ros::NodeHandle* ptr_node_handle , std::string topic_name 
                , std::string frame_name );
            
            // this use set offset value will plus in last result of relative value output
            void set_offset( double offset = 0 );

            // this use set rule for fuzzification part of error input
            void set_fuzzification_error( std::array< double, 3 >* ptr_error_rule );

            // this use set rult for fuzzification part of diff error input ( from calculate)
            void set_fuzzification_diff( std::array< double , 3 >* ptr_diff_rule );

            // this use set rule for fuzzification part of force output use to decision
            void set_fuzzification_force( std::array< double , 3 >* ptr_force_rule );

            // this use set fuzzy rule for 3Dimension
            void set_fuzzy_rule( 
                std::array< std::array < std::array < int, 7 > , 7 > , 7 >* ptr_fuzzy_rule );

            // this use defuzzification rule
            void set_defuzzification_rule( std::array< double , 3 >* ptr_defuzzy_rule );

            // use to reset system of control
            void clear_system();
            
            // use to push error or one of input system and will active process of data
            double push_error( double error );

        protected:

            // Below 4 variable will collect ptr of about fuzzification and fuzzy rule
            std::array< double , 3 >* ptr_error_rule; // rule for fuzzification error range
            std::array< double , 3 >* ptr_diff_rule; // rule for fuzzification diff range
            // force rule will have to 4 part because we want to get value for bound force
            std::array< double , 4 >* ptr_force_rule; // rule for fuzzification force range
            std::array< double , 3 >* ptr_defuzzy_rule; // rule for defuzzification relative 
            std::array< std::array < std::array < int , 7 > , 7 > , 7 >* ptr_fuzzy_rule;

            // Use to publish data
            // This message_pub will use to calculate all data in fuzzy too
            ros::Publisher fuzzy_pub;
            zeabus_utility::ControlFuzzy message_pub;

            double offset;

        private:
            // use to calculate about diff of error ( you can think that it similar velocity )
            void push_diff( double error );

            // this function will fuzzification of output
            void push_force();

            // this function will extract your input of fuzzy rule and defuzzification data
            void fuzzy_rule();

            // this function will extract of output and will publish data
            void defuzzification();

            //  this is last function will get last result and will publish all data
            //  Warning output will plus offset before
            double last_result();
           
            std::string my_name; 
    }; // object Control3Dimension

} // namespace fuzzy

} // namespace zeabus

#endif // _ZEABUS_FUZZY_CONTROL_ERROR_3DIMENSION_HPP__
