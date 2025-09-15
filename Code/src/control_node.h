#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <mutex>

#include "quadcopter.h"
#include "skidsteer.h"
#include "data_types.h"

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2/utils.h" // for getYaw
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"



/**
 * This node shows some connections and publishing images
 */


class ControllerNode : public rclcpp::Node{

public:
  /*! @brief Bar constructor.
   *
   *  Will initialise the callbacks and internal variables
   */
    ControllerNode();

  /*! @brief Bar destructor.
   *
   *  Will tear down the object
   */
    ~ControllerNode();


private:

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

  void commandTimer_callback();
  rclcpp::TimerBase::SharedPtr commandTimer_;
  bool active_;

  void activate(const std::shared_ptr<std_msgs::msg::Bool> boool);
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr activeSub_;

  Quadcopter drone_;
  Skidsteer car_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goalReady;
  geometry_msgs::msg::Twist droneCmd_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr droneCmdPub_;

  void goal_callback(const std::shared_ptr<geometry_msgs::msg::Pose> pose);
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr goalSub_;
  data::geometry_msgs::Point goal;
  data::geometry_msgs::Point convertGoalType(geometry_msgs::msg::Pose goals);

  void odo_callback(const std::shared_ptr<nav_msgs::msg::Odometry> odo);
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odoSub_;
  nav_msgs::msg::Odometry odo_;

};

