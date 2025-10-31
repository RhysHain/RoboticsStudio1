#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <cmath>
#include <thread>
#include <chrono>
#include <mutex>
#include <atomic>
#include "data_types.h"

#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/utils.h" // for getYaw
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


///@brief A struct that can store all the data of a goal
struct GoalStats {
  //! location of goal
  data::geometry_msgs::Point location;
  //! distance to goal
  double distance;
  //! time to goal
  double time;
};

/// @brief Namespace that handles the system states
namespace controller {
  typedef enum {
    WAITING,
    TURNING,
    ACCELERATING,
    BREAKING,
    LAUNCHING,
    LANDING,
    CRUISING,
    HOVER
  }MovementState;
}

/// @brief Class for controlling platforms
class Controller
{
  public:
    /// @brief Sets all members to their default value
    ///
    ///`status` is set to IDLE
    ///`tolerance_` is set to 0.5 (can be changed later @sa setTolerance)
    ///`distanceTravelled_` is set to 0
    ///`timeInMotion_` is set to 0
    ///`execute` is set to false
    Controller();

    /// @brief Begins the motion of the platform
    ///
    /// Gets the initial odometer and stores it in `initialOdo_` for later use. 
    /// `execute` is set to true
    /// `status_` is set to RUNNING
    void run(void);

    /// @brief Returns the current status of the platform
    /// @return The platform status
    data::PlatformStatus status(void);

    /// @brief Sets the goals allocated for the platform
    /// @param[in] goals The goals to be set
    /// @return Returns true if all goals are reachable from the platform's current location, going to each goal in the order supplied
    ///
    /// Each goal in `goals` is read in individually and checked if it can be completed. If the goal can be completed, it is pushed back as a GoalStats
    /// If a goal can't be completed, the function will immediately return false. It will only return true if every goal is reachable
    bool setGoals(data::geometry_msgs::Point goals);

    /// @brief Checks of the goal is rechable while also calculating the travel distance and time to the goal, as well as estimating where the platform will finish.
    /// @param[in] origin Platform start position
    /// @param[in] goal Location of the goal
    /// @param[out] distance This variable is passed by reference and will store the travel distance to the goal
    /// @param[out] time This variable is passed by reference and will store the travel time to the goal
    /// @param[out] estimatedGoalPose Estimated position the platform will be in when goal is reached
    /// @return True if the goal is reachable from origin position, false otherwise
    virtual bool checkOriginToDestination(data::nav_msgs::Odometry origin,
      data::geometry_msgs::Point goal,
      double& distance,
      double& time,
      data::nav_msgs::Odometry& estimatedGoalPose) = 0;
    
    /// @brief Returns the paltform type of the controller
    /// @return The platform type
    data::PlatformType getPlatformType(void);

    /// @brief Returns the distance between the platform and the current goal
    /// @return Distance to the current goal
    double distanceToGoal(void);

    /// @brief Returns the time remaining to reach the current goal
    /// @return Travel time remaining to current goal
    double timeToGoal(void);

    /// @brief Sets the tolerance of the platform
    /// @param[in] tolerance Value for tolerance to be set at
    /// @return True if tolerance is set, false if an invalid (value < 0) is passed through
    ///
    /// The tolerance is the distance from the goal a platform can be while still being considered at the goal
    /// A value of 0.5 is set by default
    bool setTolerance(double tolerance);

    /// @brief Returns the distance traveled since mission start
    /// @return Distance traveled
    double distanceTravelled(void);

    /// @brief Returns the time since the mission started 
    /// @return Time travelled
    double timeTravelled(void);

    /// @brief Returns the current odometry position of the platform
    /// @return Current platform odometry
    data::nav_msgs::Odometry getOdometry();

    /// @brief Gets the obstacles that the quadcopter has detected
    /// @return The detected obstacles
    ///
    /// This function has not been fully implemented as SUPER mode was not attempted
    std::vector<data::geometry_msgs::Point> getObstacles(void);
    
    /// @brief Sets the platform odometry
    /// @param[in] odo The Odometry to be set
    void setOdometry(nav_msgs::msg::Odometry odo);

    /// @brief Gets the current movement state of the platform
    /// @return The movement state
    controller::MovementState getState(void);

  protected:
    /// @brief Updates the current travel data of the platform
    ///
    /// The current position `initialOdo`, `distanceTravelled_`, and `timeInMotion_` are all updated by this function
    /// The return functions for these values simply return the stored variable
    void updateTravelData();

    //Platform goal variables
    data::nav_msgs::Odometry odo_; ///<Platform odometer
    data::nav_msgs::Odometry initialOdo_; ///<Starting odometer for certain calculations
    data::PlatformType type_; ///<Platform type
    GoalStats goals_; ///<Goals for the platform to complete
    GoalStats currentGoal; ///<Current goal the platform is moving to
    
    //Communication
    std::atomic<long> seq_; ///<Tracks the command number sent to ROS
    std::atomic<double> tolerance_; ///<Tolerance the platform will use

    //Other Variables
    std::atomic<double> distanceTravelled_; ///<Distance travelled byt he platform
    std::atomic<double> timeInMotion_; ///<Time the platform has been moving

    controller::MovementState state_; ///<Current state of the platform
    data::PlatformStatus status_; ///<Platform Status
    
    //Threading
    std::thread* thread_; ///<Thread for the controller
    std::mutex mtx; ///<Mutex for data protection
    std::atomic<bool> running_; ///<Determines if the platform should start movign towards its goals
    std::atomic<bool> execute_; ///<Determines if the platform has completed all operations
};

#endif // CONTROLLER_H
