#include "bar.h"
#include <chrono>
#include <random>

#include "tf2/utils.h" // for getYaw
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals; // Needed in the 1s wait for future

Bar::Bar() 
    : Node("bar"), marker_counter_(0) 
{
    client_ = this->create_client<std_srvs::srv::Trigger>("/detect");  

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&Bar::timer_callback, this));

    // Create a publisher for geometry_msgs::msg::Pose
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/goal", 10);

    // Create a publisher for visualization_msgs::msg::MarkerArray
    markerArray_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker", 3);

    // Finally we create a thread that will run the function threadFunction
    // The thread could start before any data is recieved via callbacks as we have not called spin yet
    // So consider this when you write your code
    thread_ = new std::thread(&Bar::threadFunction, this);

}

Bar::~Bar()
{
    // We join the thread here to make sure it is finished before the object is destroyed
    // Thanksfull, we check ros::ok() in the threadFunction to make sure it terminates, otherwise
    // we would have a deadlock here as the thread would be waiting for threadFunction to finish
    thread_->join();
}

// Here we call the service to check detection on a timer, but this could be anywhere in the code
// We could call this when we need an update of results from the service
void Bar::timer_callback(){
  
    // Wait for the service to be activated
  while (!client_->wait_for_service(1s)) {
    // If ROS is shutdown before the service is activated, show this error
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    // Print in the screen some information so the user knows what is happening
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  // Create the request, which is empty
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  service_done_ = false; 

  // Send the request to the service, create a callback to process the response
  auto result_future = client_->async_send_request(
      request, std::bind(&Bar::response_callback, this,
                          std::placeholders::_1));
}

void Bar::response_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future){
  // This will keep the response in a loop until the service is done
   while(!service_done_){
        // Check if the future has a valid response, wait for 1 second
        auto status = future.wait_for(1s);
        // If the future has a valid response, print the result, and set the service_done_ to true
        // We compare the status with the ready status
        if (status == std::future_status::ready) {
          // At the moment we are simply printing the results, but we could also use this to
          // update values in the class or do some other processing
          RCLCPP_INFO(this->get_logger(), "Result: success: %i, message: %s",
                      future.get()->success, future.get()->message.c_str());
          service_done_ = true;
        } 
        else {
          RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
        }
    }
}


void Bar::threadFunction()
{
    // This function runs in a separate thread of execution, it is started in the constructor
    // We can call any function from here, but we need to be careful with the shared resources
    // We need to make sure that we are thread safe
    // This function needs to terminate when the node is destroyed and we can do so by checking if the node is still alive
    // We will run the thread every 1s as an example
    rclcpp::Rate rate(1.0);
    while(rclcpp::ok()){
        //This function will run every second
        if (service_done_) {
            RCLCPP_INFO(this->get_logger(), "Service done, sending random pose and goal");
            
            // Publish the pose
            geometry_msgs::msg::Pose pose_msg = generateRandomPose();
            pose_publisher_->publish(pose_msg);

            visualization_msgs::msg::Marker marker = produceMarker(pose_msg);
            //We push the marker back on our array of markers
            markerArray_.markers.push_back(marker);

            //We publish the marker array
            markerArray_publisher_->publish(markerArray_);

            RCLCPP_INFO(this->get_logger(), "Published pose [x: %f, y: %f, z: %f, yaw: %f]",
                        pose_msg.position.x, pose_msg.position.y, pose_msg.position.z,
                        tf2::getYaw(pose_msg.orientation));

        }
        rate.sleep();// we sleep for 1 second as an example here
    }
    
}


geometry_msgs::msg::Pose Bar::generateRandomPose()
{
    // Create a random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dist_position(-10.0, 10.0); // Random position range [-10, 10]
    std::uniform_real_distribution<> dist_orientation(-1.0, 1.0); // Random orientation range [-1, 1]

    // Create a Pose message
    geometry_msgs::msg::Pose pose_msg;

    // Set random position
    pose_msg.position.x = dist_position(gen);
    pose_msg.position.y = dist_position(gen);
    pose_msg.position.z = dist_position(gen);

    // Set random orientation (ensure it's normalized if needed)
    pose_msg.orientation.x = dist_orientation(gen);
    pose_msg.orientation.y = dist_orientation(gen);
    pose_msg.orientation.z = dist_orientation(gen);
    pose_msg.orientation.w = dist_orientation(gen);

    // Normalize the quaternion
    double norm = std::sqrt(
        pose_msg.orientation.x * pose_msg.orientation.x +
        pose_msg.orientation.y * pose_msg.orientation.y +
        pose_msg.orientation.z * pose_msg.orientation.z +
        pose_msg.orientation.w * pose_msg.orientation.w);
    pose_msg.orientation.x /= norm;
    pose_msg.orientation.y /= norm;
    pose_msg.orientation.z /= norm;
    pose_msg.orientation.w /= norm;

    return pose_msg;
}

visualization_msgs::msg::Marker Bar::produceMarker(geometry_msgs::msg::Pose pose){

    visualization_msgs::msg::Marker marker;


    //We need to set the frame
    // Set the frame ID and time stamp.
    marker.header.frame_id = "world";
    //marker.header.seq = seq;

    marker.header.stamp = this->get_clock()->now();

    //We set lifetime (it will dissapear in this many seconds)
    marker.lifetime = rclcpp::Duration(20,0);
    // Set the namespace and id for this marker.  
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "sample";
    marker.id = marker_counter_++; //We increment the counter here to make sure we have a unique id

    // The marker type, we use a cylinder in this example
    marker.type = visualization_msgs::msg::Marker::CYLINDER;

    // Set the marker action.  Options are ADD and DELETE (we ADD it to the screen)
    marker.action = visualization_msgs::msg::Marker::ADD;

    //As an example, we are setting it
    marker.pose.position.x = pose.position.x;
    marker.pose.position.y = pose.position.y;
    marker.pose.position.z = pose.position.z;

    //Orientation, can we orientate it?
    //Could copy orientatin from pose, here settings it to 0
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0; // This is a quaternion, so we set it to 1.0 to make it a valid quaternion

    // Set the scale of the marker -- 0.5x0.5x0.5 here means 0.5m side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 0.1;

    //Alpha is stransparency (50% transparent)
    marker.color.a = 0.5f;

    //Colour is r,g,b where each channel of colour is 0-1. Bellow will make it orange
    marker.color.r = 1.0;
    marker.color.g = static_cast<float>(177.0/255.0);
    marker.color.b = 0.0;

    return marker;
}   