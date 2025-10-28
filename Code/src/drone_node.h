#include <sstream>
#include <iostream>
#include <fstream>
#include <string>

#include <mutex>
#include <queue>

#include "quadcopter.h"
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
#include <ament_index_cpp/get_package_share_directory.hpp>




class DroneNode : public rclcpp::Node{

public:
  /*! @brief Bar constructor.
   *
   *  Will initialise the callbacks and internal variables
   */
    DroneNode();

  /*! @brief Bar destructor.
   *
   *  Will tear down the object
   */
    ~DroneNode();


private:


  void commandTimer_callback();
  rclcpp::TimerBase::SharedPtr commandTimer_;
  bool active_;

  void activate(const std::shared_ptr<std_msgs::msg::Bool> boool);
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr activeSub_;

  Quadcopter drone_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goalReady;
  std_msgs::msg::Bool isReady;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr readySub_;
  void searchPattern(const std::shared_ptr<std_msgs::msg::Bool> boool);
  std::queue<data::geometry_msgs::Point> searchPatternPoints_;
  std::queue<data::geometry_msgs::Point> generateSearchPattern(bool& successful);
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr searchPatterGoalPub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr startSearchSub_;
  std_msgs::msg::Bool searching_;
  void searching(const std::shared_ptr<std_msgs::msg::Bool> boool);
  geometry_msgs::msg::Twist droneCmd_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr droneCmdPub_;

  void goal_callback(const std::shared_ptr<geometry_msgs::msg::Pose> pose);
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr goalSub_;
  data::geometry_msgs::Point goal;
  data::geometry_msgs::Point convertGoalType(geometry_msgs::msg::Pose goals);

  void pose_callback(const std::shared_ptr<geometry_msgs::msg::PoseArray> poses);
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr poseSub_;
  nav_msgs::msg::Odometry odo_;
};

