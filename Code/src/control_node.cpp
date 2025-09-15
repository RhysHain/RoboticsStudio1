#include "control_node.h"
#include <chrono>
#include <random>

#include "tf2/utils.h" // for getYaw
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals; // Needed in the 1s wait for future

ControllerNode::ControllerNode() 
    : Node("Controller_Node")
{
    activeSub_ = this->create_subscription<std_msgs::msg::Bool>("/CODES/active", 10, std::bind(&ControllerNode::activate,this, std::placeholders::_1));
    goalReady = this->create_publisher<std_msgs::msg::Bool>("/CODES/goal_ready", 10);
    droneCmdPub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    goalSub_ = this->create_subscription<geometry_msgs::msg::Pose>("/CODES/goals", 10, std::bind(&ControllerNode::goal_callback,this,std::placeholders::_1));
    odoSub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&ControllerNode::odo_callback,this,std::placeholders::_1));

    commandTimer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&ControllerNode::commandTimer_callback, this));

}

ControllerNode::~ControllerNode()
{
    
}

void ControllerNode::commandTimer_callback() {
    std_msgs::msg::Bool isReady;
    if (drone_.status() == data::PlatformStatus::IDLE) {
        isReady.data = true;
    }
    else {
        isReady.data = false;
    }
    goalReady->publish(isReady);
    
    if (active_) {
        data::commands::Quadcopter cmd = drone_.getCommands();
        droneCmd_.linear.x = cmd.move_f_b;
        droneCmd_.linear.y = cmd.move_l_r;
        droneCmd_.linear.z = cmd.move_u_d;
        droneCmd_.angular.z = cmd.turn_l_r;
        droneCmdPub_->publish(droneCmd_);
        return;
    }
    droneCmd_.linear.x = 0;
    droneCmd_.linear.y = 0;
    droneCmd_.linear.z = 0;
    droneCmd_.angular.z = 0;
    droneCmdPub_->publish(droneCmd_);
    
}

void ControllerNode::activate(const std::shared_ptr<std_msgs::msg::Bool> boool) {
    active_ = boool->data;
    if (active_) {
        RCLCPP_INFO(this->get_logger(), "Controller Activated");
    }
    else {
        RCLCPP_INFO(this->get_logger(), "Controller Deactivated");
    }
    
}

void ControllerNode::goal_callback(const std::shared_ptr<geometry_msgs::msg::Pose> pose) {
    if (active_) {
        geometry_msgs::msg::Pose goals = *pose;
        data::geometry_msgs::Point goalsToSet = convertGoalType(goals);
        if (drone_.setGoals(goalsToSet)) {
            drone_.run();
        }
    }
    RCLCPP_INFO(this->get_logger(), "Goal Received");
}

data::geometry_msgs::Point ControllerNode::convertGoalType(geometry_msgs::msg::Pose goals) {
    data::geometry_msgs::Point goalsToSet;
    goalsToSet.x = goals.position.x;
    goalsToSet.y = goals.position.y;
    goalsToSet.z = goals.position.z;
    return goalsToSet;
}
