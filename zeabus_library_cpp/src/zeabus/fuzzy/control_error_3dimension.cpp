// FILE			: control_error_3dimension.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, June 12 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE
//  ref1 : http://wiki.ros.org/rosconsole
//  ref2 : http://www.cplusplus.com/reference/cmath/signbit/

// MACRO SET

// MACRO CONDITION

#include    <zeabus/fuzzy/control_error_3dimension.hpp>

namespace zeabus
{

namespace fuzzy
{

    ControlError3Dimension::ControlError3Dimension( 
        std::shared_ptr<ros::NodeHandle> ptr_node_handle 
        , std::string topic_name 
        , std::string frame_name )
    {
        // Advertise of publish topic for record data easy to look and analysis
        this->fuzzy_pub = ptr_node_handle->advertise< zeabus_utility::ControlFuzzy >(
            topic_name, 10 );
        (this->message_pub).header.frame_id = frame_name;
        this->clear_system();
        this->my_name = topic_name;
    } // function constructor of ControlError3Dimension

    void ControlError3Dimension::clear_system()
    {
        (this->message_pub).error.crisp_data = 0;
        (this->message_pub).error.fuzzy_data = 0;
        (this->message_pub).diff.crisp_data = 0;
        (this->message_pub).diff.fuzzy_data = 0;
        (this->message_pub).force.crisp_data = 0;
        (this->message_pub).force.fuzzy_data = 0;
        (this->message_pub).output.crisp_data = 0;
        (this->message_pub).output.fuzzy_data = 0;
        (this->message_pub).result = this->offset;
        // Below function is sign to show you about reset system of fuzzy control
        (this->message_pub).header.stamp = ros::Time::now();
        (this->fuzzy_pub).publish( this->message_pub );
    } // function constuctor of clear_system

    double ControlError3Dimension::push_error( double error )
    {
        // before you save your error as input to meessage we must to calculate diff before
        this->push_diff( error );
        // Now it ture to fuzzification of error
        (this->message_pub).error.crisp_data = error;
        double temp_data = fabs( error );
        if( temp_data > this->ptr_error_rule->at(2) )
        {
            (this->message_pub).error.fuzzy_data = copysign( 3 
                    , (this->message_pub).error.crisp_data );
        }
        else if( temp_data > this->ptr_error_rule->at(1) )
        {
            (this->message_pub).error.fuzzy_data = copysign( 2 
                    , (this->message_pub).error.crisp_data );
        }
        else if( temp_data > this->ptr_error_rule->at(0) )
        {
            (this->message_pub).error.fuzzy_data = copysign( 1 
                    , (this->message_pub).error.crisp_data );
        }
        else
        {
            (this->message_pub).error.fuzzy_data = 0;
        }
        // Next I will fuzzification of force in previous turn
        this->push_force();
        // Now We already have three input data next I will use fuzzy rule to get fuzzy output
        this->fuzzy_rule();
        // Now We have fuzzy output we must to defuzzification that data
        this->defuzzification();
        // Next last step we must to get result and publish data
        return this->last_result();
    } // function push_error  


    // This will fuzzification of e(t-1) - e(t) < diff value >
    void ControlError3Dimension::push_diff( double error )
    {
        (this->message_pub).diff.crisp_data = (this->message_pub).error.crisp_data - error;
        double temp_data = fabs( (this->message_pub).diff.crisp_data );
        if( temp_data > this->ptr_diff_rule->at( 2 ) )
        {
            (this->message_pub).diff.fuzzy_data = copysign( 3 
                    , (this->message_pub).diff.crisp_data );
        }
        else if( temp_data > this->ptr_diff_rule->at( 1 ) )
        {
            (this->message_pub).diff.fuzzy_data = copysign( 2 
                    , (this->message_pub).diff.crisp_data );
        }
        else if( temp_data > this->ptr_diff_rule->at( 0 ) )
        {
            (this->message_pub).diff.fuzzy_data = copysign( 1 
                    , (this->message_pub).diff.crisp_data );
        }
        else
        {
            (this->message_pub).diff.fuzzy_data = 0;
        }
    } // function push_diff

    // This will fuzzification of force < This is previous value of output >
    void ControlError3Dimension::push_force()
    {
        (this->message_pub).force.crisp_data += (this->message_pub).output.crisp_data;
        double temp_data = fabs( (this->message_pub).force.crisp_data );
        if( temp_data > this->ptr_force_rule->at( 2 ) )
        {
            (this->message_pub).force.fuzzy_data = copysign( 3 
                    , (this->message_pub).force.crisp_data );
        }
        else if( temp_data > this->ptr_force_rule->at( 1 ) )
        {
            (this->message_pub).force.fuzzy_data = copysign( 2 
                    , (this->message_pub).force.crisp_data );
        }
        else if( temp_data > this->ptr_force_rule->at( 0 ) )
        {
            (this->message_pub).force.fuzzy_data = copysign( 1 
                    , (this->message_pub).force.crisp_data );
        }
        else
        {
            (this->message_pub).force.fuzzy_data = 0;
        }
    } // function push_force

    void ControlError3Dimension::fuzzy_rule()
    {
        // We get data fuzzy from our rule
        (this->message_pub).output.fuzzy_data = 
            (this->ptr_fuzzy_rule->at( 
                (this->message_pub).force.fuzzy_data + 3 )).at(
                    (this->message_pub).diff.fuzzy_data + 3 ).at(
                        (this->message_pub).error.fuzzy_data + 3 );
        std::cout << "original data of output " << (this->message_pub).output.fuzzy_data << "\n";
    }

    void ControlError3Dimension::defuzzification()
    {
        // Now we have to consider about fuzzy output
        short int temp_fuzzy = abs( ( this->message_pub ).output.fuzzy_data );
        ROS_WARN_COND( temp_fuzzy == 6 , "%s I thinsk it impossible case" 
            , this->my_name.c_str() );

        if( temp_fuzzy == 0 )
        {
            (this->message_pub).output.crisp_data = 0;
        }
        else if( temp_fuzzy < 6 )
        {
            (this->message_pub).output.crisp_data = 
                copysign(this->ptr_defuzzy_rule->at( temp_fuzzy - 1 ) 
                    , (this->message_pub).output.fuzzy_data );
        }
        else
        {
            (this->message_pub).output.crisp_data = 
                copysign(this->ptr_defuzzy_rule->at( 4 ) 
                    , (this->message_pub).output.fuzzy_data );
        }

    } //  function defuzzification

    double ControlError3Dimension::last_result()
    {
        (this->message_pub).result = this->offset 
            + (this->message_pub).force.crisp_data
            + (this->message_pub).output.crisp_data;
        (this->fuzzy_pub).publish( this->message_pub );
        return (this->message_pub).result;
    }

    void ControlError3Dimension::set_fuzzy_rule(
        const std::array< std::array < std::array < int , 7 > , 7 > , 7 >* ptr_fuzzy_rule )
    {
        this->ptr_fuzzy_rule = ptr_fuzzy_rule;
    }

    void ControlError3Dimension::set_offset( const double offset )
    {
        this->offset = offset;
    }

    void ControlError3Dimension::set_fuzzification_error( 
        const std::array< double, 3>* ptr_error_rule )
    {
        this->ptr_error_rule = ptr_error_rule;
    }

    void ControlError3Dimension::set_fuzzification_diff( 
        const std::array< double, 3>* ptr_diff_rule )
    {
        this->ptr_diff_rule = ptr_diff_rule;
    }

    void ControlError3Dimension::set_fuzzification_force( 
        const std::array< double, 4 >* ptr_force_rule )
    {
        this->ptr_force_rule = ptr_force_rule;
    }

    void ControlError3Dimension::set_defuzzification_rule( 
        const std::array< double, 5 >* ptr_defuzzy_rule )
    {
        this->ptr_defuzzy_rule = ptr_defuzzy_rule;
    }

} // namespace fuzzy

} // namespace zeabus
