// FILE			: path_file.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 10 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README
//  We can't check your path already exsits because we have to limit compile version to c++14
//  Header filesystem can use in c++17

// REFERENCE
//  https://en.cppreference.com/w/cpp/filesystem/exists

#include    <zeabus/ros_interfaces/file/path_file.hpp>

namespace zeabus
{

namespace ros_interfaces
{

namespace file
{

    PathFile::PathFile( std::string full_path )
    {
        this->path_package = "";
        this->path_subdirectory = "";
        this->file_name = "";
        this->full_path = full_path;
    } // function constructor

    bool PathFile::setup_package( std::string package_name )
    {
        bool result = true;
        this->path_package = ros::package::getPath( package_name );
        // if above function don't found package. That function will return empty string
        if( this->path_package == "" )
        {
            std::cout   << zeabus::escape_code::bold_red << "Don't find package "
                        << package_name << std::endl << zeabus::escape_code::normal_white;
            result = false;
        }
#ifdef _PRINT_PROCESS_
        else
        {
            std::cout   << "Find your pacakge path is " << this->path_package << std::endl;
        }
#endif
        return result;
    } // function setup_pacakge
    
    void PathFile::setup_subdirectory( std::string subdirectory )
    {
        this->path_subdirectory = subdirectory;
    } // function setup_subdirectory

    void PathFile::setup_file_name( std::string file_name )
    {
        this->file_name = file_name;
    }

    bool PathFile::updated_path( )
    {
        bool result = true;
        if( this->path_package == "" )
        {
            result = false;
            std::cout   << "Please setup your path to package\n";
        }
        else
        {
            this->full_path = this->path_package;
        }
        if( this->path_subdirectory == "")
        {
            result = false;
            std::cout   << "please setup your subdirectory\n";
        }
        else if( result )
        {
            this->full_path = this->full_path + "/" + this->path_subdirectory;
        }
        else
        {
            ; // can't do because above condition are false
        }
        if( this->file_name == "" )
        {
            result = false;
            std::cout   << "please setup your file name\n";
        }
        else if( result )
        {
            this->full_path = this->full_path + "/" + this->file_name;
        }
        else
        {
            ; // can't do because above condition are false
        }
        return result;
    }

    std::string PathFile::get_full_path()
    {
        return this->full_path;
    }

} // namespace File

} // namespace ros_interfaces

} // namespace zeabus
