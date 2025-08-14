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
class Foo : public rclcpp::Node{

public:
  /*! @brief Foo constructor.
   *
   *  Will initialise the callbacks and internal variables
   */
    Foo();

  /*! @brief Foo destructor.
  *
  *  Will tear down the object
  */
  ~Foo();


/*! @brief - A callback for the service
  *  
  * @param[in] req - The request object (not used as it is empty in this service call type)
  * @param[out] res - The response object which contains a number of valid readings in last laser scan received "Readings: <number of readings>"
  * if no laser scan is received, the response will contain error message "ERROR: No laser data available"
  */
  void detect(const std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

private:

  /*! @brief - A callback for the timer
  * We will simply do some logging here in this function as an example
  */
  void timerCallback();

  /*! @brief - A function that will be run in a separate thread
  * We will simply do some logging here in this function as an example
  */
  void threadFunction();

  /*! @brief LaserScan Callback
  *
  *  @param std::shared_ptr<sensor_msgs::msg::LaserScan - The laserscan message as a const pointer
  *  @note This function and the declaration are ROS specific to handle callbacks
  */
  void laserCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg);

  /*! @brief Pose Array Callback
  *
  *  @param std::shared_ptr<geometry_msgs::msg::Pose> - The message as a const pointer
  *  @note This function and the declaration are ROS specific to handle callbacks
  */
 void poseCallback(const std::shared_ptr<geometry_msgs::msg::Pose> msg);

private:

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_; //!< Pointer to the service object 
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub1_;//!< Pointer to the laser scan subscriber
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub2_;//!< Pointer to the pose subscriber
    std::unique_ptr<LaserProcessing> laserProcessingPtr_;//!< Pointer to the laser processing object
    std::thread* thread_; //!< Thread object pointer
    rclcpp::TimerBase::SharedPtr timer_; //!< Timer object pointer, will be used to run a function at regular intervals

};

