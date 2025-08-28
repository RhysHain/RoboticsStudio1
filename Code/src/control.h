#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "laserprocessing.h"


/**
 * This node shows some connections and publishing images
 */


class Control : public rclcpp::Node{

public:
  /*! @brief Bar constructor.
   *
   *  Will initialise the callbacks and internal variables
   */
    Control();

  /*! @brief Bar destructor.
   *
   *  Will tear down the object
   */
    ~Control();


private:

  /*! @brief - Function invoke by the timer
   * We will simply print to screen some information
   */
  void timer_callback();

  /*! @brief - A responce for the callback
   *  
   * @param[in] future - future object that will be used to get the response
   * We will use this function to be called when the service sends a response
   * Checking the result (ready state) and printing them to the screen
   */
  void response_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);

  /*! @brief - A function that will be run in a separate thread
  * It runs at 1Hz, if the service is done, it will generate a random pose and publish it
  * It will also generate a marker and publish it
  */
 void threadFunction();


private:


};

