// FILE			: fuzzy_3_dimension.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, July 15 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE
//  ref01   : http://www.cplusplus.com/reference/iomanip/setw/
//  ref02   : http://www.cplusplus.com/reference/iomanip/setprecision/

// MACRO SET

// MACRO CONDITION

#include    <array>

#include    <iostream>

#include    <memory>

#include    <iomanip>

#include    <ros/ros.h>

#include    <zeabus_control/fuzzy/all_parameter.hpp>

#include    <zeabus/fuzzy/control_error_3dimension.hpp>

int main( int argv , char** argc )
{

    ros::init( argv , argc , "testing_fuzzy" );

    const std::array< double , 3 >* error_rule = 
        &zeabus_control::fuzzy::yaw_parameter::ERROR_RULE;

    const std::array< double , 3 >* diff_rule = 
        &zeabus_control::fuzzy::yaw_parameter::DIFF_RULE;

    const std::array< double , 4 >* force_rule = 
        &zeabus_control::fuzzy::yaw_parameter::FORCE_RULE;

    const std::array< double , 5 >* defuzz_rule = 
        &zeabus_control::fuzzy::yaw_parameter::DEFUZZY_RULE;

    const std::array< std::array< std::array< int , 7 > , 7 > , 7 >* fuzzy_rule=
        &zeabus_control::fuzzy::yaw_parameter::FUZZY_RULE;

    char input_char;
    double input_error;

    std::cout   << "You want to look and check individual rule? [y , n] : ";
    std::cin    >> input_char;
    
    std::cout   << std::setprecision(3) << std::setw( 10 );
    switch( input_char )
    {
        case 'y' :
        case 'Y' :
            std::cout   << "ERROR rule      :";
            for( unsigned int run = 0 ; run < 3 ; run++ )
            {
                std::cout   << " " << error_rule->at( run );
            }
            std::cout   << "\nDIFF rule       :";
            for( unsigned int run = 0 ; run < 3 ; run++ )
            {
                std::cout   << " " << diff_rule->at( run );
            }
            std::cout   << "\nFORCE rule      :";
            for( unsigned int run = 0 ; run < 4 ; run++ )
            {
                std::cout   << " " << force_rule->at( run );
            }
            std::cout   << "\nDEFUZZ rule     :";
            for( unsigned int run = 0 ; run < 5 ; run++ )
            {
                std::cout   << " " << defuzz_rule->at( run );
            } 
            std::cout   << "\n";
        case 'n' :
        case 'N' :
            std::cout   << "==========================";
            std::cout   << "You want to look all rule? [y , n]";
            std::cin    >> input_char;
            std::cout   << std::setw(4);
            switch( input_char )
            {
                case 'y':
                case 'Y':
                    for( unsigned int force_run = 0 ; force_run < 7 ; force_run++ )
                    {
                        std::cout   << "\nTable in case force is " << force_run - 3 << "\n";
                        for( unsigned int diff_run = 0 ; diff_run < 7 ; diff_run++ )
                        {
                            std::cout   << "\t" ;
                            for( unsigned int error_run = 0 ; error_run < 7 ; error_run++ )
                            {
                                std::cout   << " " <<fuzzy_rule->at( 
                                                force_run ).at( diff_run ).at(error_run);
                            }
                            std::cout   << "\n";
                        }
                    }
                case 'n':
                case 'N':
                    break;
                default :
                    std::cout   << "Input error!!\n";
                    ros::shutdown();
            }
            break;
        default:
            std::cout   << "Input error!!\n";
            ros::shutdown();
    } // switch char

    std::shared_ptr< ros::NodeHandle > ptr_node_handle =
            std::make_shared< ros::NodeHandle >("");

    zeabus::fuzzy::ControlError3Dimension fuzzy_yaw( ptr_node_handle, "/control/yaw" , "odom");

}
