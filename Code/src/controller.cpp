#include "controller.h"


Controller::Controller() {
    status_ = data::PlatformStatus::IDLE;
    tolerance_ = 0.5;
    distanceTravelled_ = 0;
    timeInMotion_ = 0;
    seq_ = 0;

    execute_ = false;

    goals_.location.x = 0;
}

void Controller::run(void) {
    initialOdo_ = this->getOdometry();
    execute_ = true;
    status_ = data::PlatformStatus::RUNNING;
}

data::PlatformStatus Controller::status(void) {
    return status_;
}

bool Controller::setGoals(data::geometry_msgs::Point goals) {
    data::nav_msgs::Odometry odo = this->getOdometry();
        data::nav_msgs::Odometry endPose;
        if (this->checkOriginToDestination(odo, goals, goals_.distance, goals_.time, endPose)) {
            goals_.location.x = endPose.position.x;
            goals_.location.y = endPose.position.y;
            goals_.location.z = endPose.position.z;
        }
        else {
            return false;
        }
    
    return true;
}

data::PlatformType Controller::getPlatformType(void) {
    return type_;
}

double Controller::distanceToGoal(void) {
    if (goals_.location.x == 0) {
        return 0;
    }
    data::nav_msgs::Odometry goalPose;
    this->checkOriginToDestination(this->getOdometry(), currentGoal.location, currentGoal.distance, currentGoal.time, goalPose);
    return currentGoal.distance;
}
double Controller::timeToGoal(void) {
    if (goals_.location.x == 0) {
        return 0;
    }
    data::nav_msgs::Odometry goalPose;
    this->checkOriginToDestination(this->getOdometry(), currentGoal.location, currentGoal.distance, currentGoal.time, goalPose);
    return currentGoal.time;
}

bool Controller::setTolerance(double tolerance) {
  if (tolerance < 0) {
    return false;
  }
  tolerance_ = tolerance;
  return true;
}

double Controller::distanceTravelled(void) {
    return distanceTravelled_;
}
double Controller::timeTravelled(void) {
    return timeInMotion_;
}

data::nav_msgs::Odometry Controller::getOdometry() {
    return odo_;
}

void Controller::setOdometry(nav_msgs::msg::Odometry odo) {
    odo_.position.x = odo.pose.pose.position.x;
    odo_.position.y = odo.pose.pose.position.y;
    odo_.position.z = odo.pose.pose.position.z;

    odo_.linear.x = odo.twist.twist.linear.x;
    odo_.linear.y = odo.twist.twist.linear.y;
    odo_.linear.z = odo.twist.twist.linear.z;
    odo_.yaw = tf2::getYaw(odo.pose.pose.orientation);
}

std::vector<data::geometry_msgs::Point> Controller::getObstacles(void) {
    std::vector<data::geometry_msgs::Point> obstacles;
    
    return obstacles;
}

void Controller::updateTravelData() {
    data::nav_msgs::Odometry odo = this->getOdometry();
    distanceTravelled_ = distanceTravelled_ + sqrt(pow(odo.position.x - initialOdo_.position.x, 2) + pow(odo.position.y - initialOdo_.position.y, 2));
    timeInMotion_ = timeInMotion_ + odo.time - initialOdo_.time;
    initialOdo_ = odo;
}