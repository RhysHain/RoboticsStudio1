#include <sstream>
#include <iostream>
#include <fstream>
#include <string>

#include <mutex>
#include <queue>
#include <atomic>

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



/// @brief ROS2 Humble node for controlling the drone
class DroneNode : public rclcpp::Node{

public:
  /// @brief Constructor for Drone Node
  ///
  /// Publishers, subscribers and timers are initilised
  DroneNode();

  /// @brief Destructor for Drone Node
  ///
  /// No functionality, included for completeness
  ~DroneNode();


private:
  /// @brief Callback for the command timer. It runs at 20Hz
  ///
  /// Despite the name, this function also checks the current drone status, publishing True through `goalReady` is the drone is Idle, False otherwise
  /// If `active` is True, commands are read from `drone_` and puclished to the drone. Otherwise, a deafult set of commands published
  void commandTimer_callback();

  /// @brief Callback for the debug timer. It runs at 2Hz
  ///
  /// Has no function in the system, can be modified to log debug messages based on system states
  void debugTimer_callback();

  /// @brief Callback for the drone activation on topic /CODES/parrot/active
  /// @param[in] boool Value from ROS
  /// Will activate or deactivate the drone, based on what had been published to the topic
  void activate(const std::shared_ptr<std_msgs::msg::Bool> boool);

  /// @brief Callback to send the next goal to the drone, on topic /CODES/parrot/goal_ready
  /// @param[in] boool Value fron ROS
  /// If the data is true and the search pattern has been started, the next goal in the search pattern will be published
  void searchPattern(const std::shared_ptr<std_msgs::msg::Bool> boool);

  /// @brief Generates the search pattern from a given file
  /// @param[out] successful If the search pattern has been genereated succesfully
  /// @return The search pattern
  /// Reads from the file searchPatternPoints.txt in the config folder
  /// The format of the coordinates is x y z single space single line
  /// Bad lines will be automatically ignored
  std::queue<data::geometry_msgs::Point> generateSearchPattern(bool& successful);

  /// @brief Callback for starting the search pattern on topic /CODES/parrot/start_search
  /// @param[in] boool Data from ROS
  void searching(const std::shared_ptr<std_msgs::msg::Bool> boool);

  /// @brief Callback for sensing goals to the drone
  /// @param[in] pose Pose of the goal to be sent
  /// If the drone is valid (can be reached by the drone) the drone will be sent to the goal
  void goal_callback(const std::shared_ptr<geometry_msgs::msg::Pose> pose);

  /// @brief Callback for obtaining the pose of the drone on topic /parrot/pose
  /// @param[in] poses The drone pose
  /// Updates the `odo_` member and sends the new odometry to the drone
  /// This is required as the odometry from gazebo only gives a 2D pose
  void pose_callback(const std::shared_ptr<geometry_msgs::msg::PoseArray> poses);

  /// @brief Callback for obtaining the odometry of the drone on topic /parrot/odometry
  /// @param[in] odo The odometry from ROS
  /// Updates the `odo_` member
  void odo_callback(const std::shared_ptr<nav_msgs::msg::Odometry> odo);

  /// @brief Converts the goal structure from ROS to our internal data structure
  /// @param[in] goals The goal to be converted
  /// @return The convereted goal
  data::geometry_msgs::Point convertGoalType(geometry_msgs::msg::Pose goals);
  
  // Publishers
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goalReady; ///< Publishes if the drone is ready for the next goal to /CODES/parrot/goal_ready
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr searchPatterGoalPub_; ///< Publishes the next search pattern goal to /CODES/parrot/goals
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr droneCmdPub_; ///< Publishes the drone commands to /parrot/cmd_vel

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr activeSub_; ///< Sunscribes to /CODES/parrot/active
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr readySub_; ///< Subscribes to /CODES/parrot/goal_ready
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr startSearchSub_; ///< Subscribes to /CODES/parrot/start_search
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr goalSub_; ///< Subscribes to CODES/parrot/goals
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr poseSub_; ///< Subscribes to /parrot/pose
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odoSub_; ///< Subscribes to /parrot/odometry

  // Timers
  rclcpp::TimerBase::SharedPtr commandTimer_; ///< Command Timer
  rclcpp::TimerBase::SharedPtr debugTimer_; ///C Debug output timer

  // Other Private Members
  bool active_; ///< Stores if the node is active or not
  Quadcopter drone_; ///< The drone object
  std_msgs::msg::Bool isReady; ///< Stores of the drone is ready for a goal
  std::queue<data::geometry_msgs::Point> searchPatternPoints_; ///< The stored search pattern
  std_msgs::msg::Bool searching_; ///< Stores if the system should be using the search pattern
  geometry_msgs::msg::Twist droneCmd_; ///< Stores the drone commands
  data::geometry_msgs::Point goal; ///< Stores the current goal
  std::atomic<double> z_; ///< Stores the z coordinate of the drone
  nav_msgs::msg::Odometry odo_; ///< Stores the odometry of the drone
};

