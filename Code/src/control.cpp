#include "control.h"
#include <chrono>
#include <random>

#include "tf2/utils.h" // for getYaw
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals; // Needed in the 1s wait for future

Control::Control() 
    : Node("bar"), marker_counter_(0) 
{
    
}

Control::~Control()
{
    
}


void Control::timer_callback(){
  
}

void Control::response_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future){

}


void Control::threadFunction()
{
    
}
