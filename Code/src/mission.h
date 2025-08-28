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

// We have included the laserProcessing header file here as it is used in the node
#include "laserprocessing.h"


/*!
* @brief Foo class. 
* This is the class that will be used to create the node, it needs to inherit from rclcpp::Node
* This class will have the callbacks for the services and the subscribers
* Would recommend that processing is done in libraries and not in the node itself, the example here is laserProcessing
*/
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

