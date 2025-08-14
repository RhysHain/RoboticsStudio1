#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "laserprocessing.h"


/**
 * This node shows some connections and publishing images
 */


class Bar : public rclcpp::Node{

public:
  /*! @brief Bar constructor.
   *
   *  Will initialise the callbacks and internal variables
   */
    Bar();

  /*! @brief Bar destructor.
   *
   *  Will tear down the object
   */
    ~Bar();


private:

  /*! @brief - Function invoke by the timer
   * We will simply print to screen some information
   */
  void timer_callback();

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

  /*! @brief - Generate a random pose
  * The pose is generated randomly in a range of -10 to 10 for x, y and z
  * The orientation is generated randomly in a range of -1 to 1 for x, y, z and w
  * The pose is then normalized to ensure it is a valid quaternion
  */
 geometry_msgs::msg::Pose generateRandomPose();

    /*! @brief Obtain Marker of CUBE from geometry_msgs::Pose
    * The markers are reported in world coordinate frames, namespace sample, type CUBE, colour orange, valid for 20s
    *
    *  @param pose - a geometry_msgs::Pose object
    *  @return marker - a visualization_msgs::msg::Marker object
    */
  visualization_msgs::msg::Marker produceMarker(geometry_msgs::msg::Pose pose);


private:

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerArray_publisher_;

  rclcpp::TimerBase::SharedPtr timer_; //!< Timer object pointer, will be used to run a function at regular intervals
  bool service_done_ = false; //<!< Flag to check if the service is done
  std::thread* thread_; //!< Thread object pointer

  visualization_msgs::msg::MarkerArray markerArray_; //!< Marker array object, markers get added to this array
  unsigned int marker_counter_; //!< Marker counter


};

