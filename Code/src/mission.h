#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"

// In the example we only have laser scan and geometry_msg and trigger services
// Add the messages and services you need here 
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_srvs/srv/trigger.hpp"



class Mission : public rclcpp::Node{

public:
  /*! @brief Foo constructor.
   *
   *  Will initialise the callbacks and internal variables
   */
    Mission();

  /*! @brief Foo destructor.
  *
  *  Will tear down the object
  */
  ~Mission();


private:

  /*! @brief - A callback for the timer
  * We will simply do some logging here in this function as an example
  */
  void timerCallback();

  /*! @brief - A function that will be run in a separate thread
  * We will simply do some logging here in this function as an example
  */
  void threadFunction();


private:


};

