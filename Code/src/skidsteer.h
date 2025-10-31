#ifndef SKIDSTEER_H
#define SKIDSTEER_H

#include "controller.h"

/// @brief Class for controlling a skidsteer
class Skidsteer: public Controller
{
public:
  /// @brief Constructor for the Quadcopter
  ///
  /// Thread is started and the platform type is defined. The ROS connector is also defined here
  Skidsteer();

  /// @brief Quadcopter Destructor
  ///
  /// Thread is joined and a stop command is sent to the platform
  ~Skidsteer();
  
  /// @brief Checks of the goal is rechable while also calculating the travel distance and time to the goal, as well as estimating where the platform will finish.
  /// @param[in] origin Platform start position
  /// @param[in] goal Location of the goal
  /// @param[out] distance This variable is passed by reference and will store the travel distance to the goal
  /// @param[out] time This variable is passed by reference and will store the travel time to the goal
  /// @param[out] estimatedGoalPose Estimated position the platform will be in when goal is reached
  /// @return True if the goal is reachable from origin position, false otherwise
  ///
  /// Uses trigonometry to determine the distance the goal is away, and what angle turn is required to face the goal
  /// Since the paltform only moves at 1m/s and turns at 1rad/s, the angle and the distance can be summed for the move time
  /// The quadcopter can always reach the goal, so the function always returns true
  bool checkOriginToDestination(data::nav_msgs::Odometry origin,
    data::geometry_msgs::Point goal,
    double& distance,
    double& time,
    data::nav_msgs::Odometry& estimatedGoalPose);

  data::commands::SkidSteer getCommands();

private: 
  /// @brief Executes the movement operation for the Quadcopter
  ///
  /// Runs through each goal that has been set and passing them through the @sa move() function
  /// Once completed, it stops the loop the thread is running
  void reachGoal(void);

  
  void sendCmd(double turn_l_r, double move_f_b);

  /// @brief Calculates the steering angle the quadcopter must use
  /// @param[in] odo Current platform odometry
  /// @param[in] goal Current goal
  /// @param[out] steering Steering angle, passed by reference
  /// @param[in] goalAngle Relative angle to the goal from the quadcopter
  /// @param[out] turn Turn speed required, passed by reference
  /// @return True if the calculation can be completed, false otherwise
  bool computerSteering(data::nav_msgs::Odometry odo, data::geometry_msgs::Point goal, double& steering, double goalAngle, double& turn);

  /// @brief Moves the platform to the desired goal
  /// @param[in] stats The goal to move to
  /// @return True if the goal has been reached, false if the goal can't be reached
  bool move(GoalStats stats);

  data::commands::SkidSteer cmd_;
};

#endif // QUADCOPTER_H
