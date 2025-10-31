#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <mutex>

#include "skidsteer.h"
#include "data_types.h"

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/utils.h" // for getYaw
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"



/// @brief ROS2 Humble node for controlling the husky
class HuskyNode : public rclcpp::Node{

public:
  /*! @brief Bar constructor.
   *
   *  Will initialise the callbacks and internal variables
   */
    HuskyNode();

  /*! @brief Bar destructor.
   *
   *  Will tear down the object
   */
    ~HuskyNode();


private:


  void commandTimer_callback();
  rclcpp::TimerBase::SharedPtr commandTimer_;
  bool active_;

  void activate(const std::shared_ptr<std_msgs::msg::Bool> boool);
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr activeSub_;

  Skidsteer husky_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goalReady;
  geometry_msgs::msg::Twist huskyCmd_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr huskyCmdPub_;

  void goal_callback(const std::shared_ptr<geometry_msgs::msg::Pose> pose);
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr goalSub_;
  data::geometry_msgs::Point goal;
  data::geometry_msgs::Point convertGoalType(geometry_msgs::msg::Pose goals);

  void odo_callback(const std::shared_ptr<nav_msgs::msg::Odometry> odo);
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odoSub_;
  nav_msgs::msg::Odometry odo_;

};

