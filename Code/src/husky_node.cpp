#include "husky_node.h"
#include <chrono>
#include <random>

#include "tf2/utils.h" // for getYaw
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals; // Needed in the 1s wait for future

HuskyNode::HuskyNode() 
    : Node("Drone_Node")
{
    activeSub_ = this->create_subscription<std_msgs::msg::Bool>("/CODES/husky/active", 10, std::bind(&HuskyNode::activate,this, std::placeholders::_1));
    goalReady = this->create_publisher<std_msgs::msg::Bool>("/CODES/husky/goal_ready", 10);
    huskyCmdPub_ = this->create_publisher<geometry_msgs::msg::Twist>("/CODES/husky/cmd_vel", 10);
    goalSub_ = this->create_subscription<geometry_msgs::msg::Pose>("/CODES/husky/goals", 10, std::bind(&HuskyNode::goal_callback,this,std::placeholders::_1));
    odoSub_ = this->create_subscription<nav_msgs::msg::Odometry>("/CODES/husky/odom", 10, std::bind(&HuskyNode::odo_callback,this,std::placeholders::_1));

    commandTimer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&HuskyNode::commandTimer_callback, this));

}

HuskyNode::~HuskyNode()
{
    
}

void HuskyNode::commandTimer_callback() {
    std_msgs::msg::Bool isReady;
    if (husky_.status() == data::PlatformStatus::IDLE) {
        isReady.data = true;
    }
    else {
        isReady.data = false;
    }
    goalReady->publish(isReady);
    
    if (active_) {
        data::commands::SkidSteer cmd = husky_.getCommands();
        huskyCmd_.linear.x = cmd.move_f_b;
        huskyCmd_.linear.y = 0;
        huskyCmd_.linear.z = 0;
        huskyCmd_.angular.z = cmd.turn_l_r;
        huskyCmdPub_->publish(huskyCmd_);
        return;
    }
    huskyCmd_.linear.x = 0;
    huskyCmd_.linear.y = 0;
    huskyCmd_.linear.z = 0;
    huskyCmd_.angular.z = 0;
    huskyCmdPub_->publish(huskyCmd_);
    
}

void HuskyNode::activate(const std::shared_ptr<std_msgs::msg::Bool> boool) {
    active_ = boool->data;
    if (active_) {
        RCLCPP_INFO(this->get_logger(), "Husky Activated");
    }
    else {
        RCLCPP_INFO(this->get_logger(), "Husky Deactivated");
    }
    
}

void HuskyNode::goal_callback(const std::shared_ptr<geometry_msgs::msg::Pose> pose) {
    if (active_) {
        geometry_msgs::msg::Pose goals = *pose;
        data::geometry_msgs::Point goalsToSet = convertGoalType(goals);
        if (husky_.setGoals(goalsToSet)) {
            husky_.run();
        }
    }
    RCLCPP_INFO(this->get_logger(), "Husky Goal Received");
}

data::geometry_msgs::Point HuskyNode::convertGoalType(geometry_msgs::msg::Pose goals) {
    data::geometry_msgs::Point goalsToSet;
    goalsToSet.x = goals.position.x;
    goalsToSet.y = goals.position.y;
    goalsToSet.z = goals.position.z;
    return goalsToSet;
}

void HuskyNode::odo_callback(const std::shared_ptr<nav_msgs::msg::Odometry> odo) {
    odo_ = *odo;
    husky_.setOdometry(odo_);
}