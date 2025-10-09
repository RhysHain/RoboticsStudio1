#include "drone_node.h"
#include <chrono>
#include <random>

#include "tf2/utils.h" // for getYaw
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals; // Needed in the 1s wait for future

DroneNode::DroneNode() 
    : Node("Drone_Node")
{
    activeSub_ = this->create_subscription<std_msgs::msg::Bool>("/CODES/parrot/active", 10, std::bind(&DroneNode::activate,this, std::placeholders::_1));
    goalReady = this->create_publisher<std_msgs::msg::Bool>("/CODES/parrot/goal_ready", 10);
    droneCmdPub_ = this->create_publisher<geometry_msgs::msg::Twist>("/CODES/parrot/cmd_vel", 10);
    goalSub_ = this->create_subscription<geometry_msgs::msg::Pose>("CODES/parrot/goals", 10, std::bind(&DroneNode::goal_callback,this,std::placeholders::_1));
    odoSub_ = this->create_subscription<nav_msgs::msg::Odometry>("/CODES/parrot/odometry", 10, std::bind(&DroneNode::odo_callback,this,std::placeholders::_1));
    readySub_ = this->create_subscription<std_msgs::msg::Bool>("/CODES/parrot/goal_ready", 10, std::bind(&DroneNode::searchPattern,this, std::placeholders::_1));
    startSearchSub_ = this->create_subscription<std_msgs::msg::Bool>("/CODES/parrot/start_search", 10, std::bind(&DroneNode::searching,this, std::placeholders::_1));
    searchPatterGoalPub_ = this->create_publisher<geometry_msgs::msg::Pose>("/CODES/parrot/goals", 10);
    commandTimer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&DroneNode::commandTimer_callback, this));

    active_ = false;
    isReady.data = false;

    //Populate the search pattern queue

}

DroneNode::~DroneNode()
{
    
}

void DroneNode::commandTimer_callback() {
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

void DroneNode::activate(const std::shared_ptr<std_msgs::msg::Bool> boool) {
    bool previous = active_;
    active_ = boool->data;
    if (active_) {
        if (previous) {
            RCLCPP_INFO(this->get_logger(), "Drone Already Activated");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Drone Activated");
    }
    else {
        if (previous) {
            RCLCPP_INFO(this->get_logger(), "Drone Deactivated");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Drone Already Deactivated");
    }
    
}

void DroneNode::goal_callback(const std::shared_ptr<geometry_msgs::msg::Pose> pose) {
    if (active_) {
        if (isReady.data) {
            geometry_msgs::msg::Pose goals = *pose;
            data::geometry_msgs::Point goalsToSet = convertGoalType(goals);
            if (drone_.setGoals(goalsToSet)) {
                drone_.run();
            }
            RCLCPP_INFO(this->get_logger(), "Drone Goal Received");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "No Goal Sent, Drone is busy");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "No goal sent, Drone is not active");
}

data::geometry_msgs::Point DroneNode::convertGoalType(geometry_msgs::msg::Pose goals) {
    data::geometry_msgs::Point goalsToSet;
    goalsToSet.x = goals.position.x;
    goalsToSet.y = goals.position.y;
    goalsToSet.z = goals.position.z;
    return goalsToSet;
}

void DroneNode::odo_callback(const std::shared_ptr<nav_msgs::msg::Odometry> odo) {
    odo_ = *odo;
    drone_.setOdometry(odo_);
}

void DroneNode::searching(const std::shared_ptr<std_msgs::msg::Bool> boool) {
    searching_ = *boool;
    isReady.data = boool->data;
    goalReady->publish(isReady);
    if (boool->data) {
        RCLCPP_INFO(this->get_logger(), "Search Pattern Started");
        while (!searchPatternPoints_.empty()) {
            searchPatternPoints_.pop();
        }
        searchPatternPoints_ = generateSearchPattern();
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Search Pattern Stopped");
}

void DroneNode::searchPattern(const std::shared_ptr<std_msgs::msg::Bool> boool) {
    if (boool->data && searching_.data) {
        geometry_msgs::msg::Pose goal;
        data::geometry_msgs::Point nextPoint = searchPatternPoints_.front();
        searchPatternPoints_.pop();
        goal.position.x = nextPoint.x;
        goal.position.y = nextPoint.y;
        goal.position.z = nextPoint.z;
        searchPatterGoalPub_->publish(goal);
        std::stringstream message;
        RCLCPP_INFO(this->get_logger(), "Search Pattern Point [%.2f, %.2f, %.2f] Has Been Sent", nextPoint.x, nextPoint.y, nextPoint.z);
    }
}

std::queue<data::geometry_msgs::Point> DroneNode::generateSearchPattern() {
    std::queue<data::geometry_msgs::Point> points;

    std::ifstream infile("searchPatternPoints.txt");
    if (!infile.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Error: could not open Search Pattern file.");
    }

    std::string line;

    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        data::geometry_msgs::Point p;
        if (!(iss >> p.x >> p.y >> p.z)) {
            std::cerr << "Warning: invalid line -> " << line << std::endl;
            continue;  // skip malformed lines
        }
        points.push(p);
    }

    infile.close();

    return points;
}