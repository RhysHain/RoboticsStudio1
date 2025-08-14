// These headers are essential for the test
#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/reader.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

// These headers are needed for the code being tested as we use laser data
#include "sensor_msgs/msg/laser_scan.hpp"
// You will need to add headers for any other messages that you use

// Include the header file for the class that you are testing
// relative to this file
#include "../src/laserprocessing.h"

// We state the test suite name and the test name
TEST(LaserProcessing,CountReturns){

    // Reading bag files based on https://docs.ros.org/en/humble/Tutorials/Advanced/Reading-From-A-Bag-File-CPP.html

    // We read the bag file to get the messages
    // The bag is in the data folder of the package
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("a3_skeleton");//If you chnage package name you need to change this
    std::string bag_filename=package_share_directory + "/data/position1"; // The data for the bag are in the data folder and each folder (such as position1) has a bag file

    rosbag2_cpp::Reader reader; // Create a reader object

    // For each message type we need a serialization object specifci ti data type
    rclcpp::Serialization<sensor_msgs::msg::LaserScan> serialization;
    // We also need a message object (shared pointer in this case)
    sensor_msgs::msg::LaserScan::SharedPtr laser_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

    // Open the bag file
    reader.open(bag_filename);
    // We will loop the messages until we find one of the topic we are interested in and then we will process it
    // Below we break the loop as soon as we have the one message
    // If you need multiple messages you need to adjust this loop to break when your have both messages
    while (reader.has_next()) {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();

        if (msg->topic_name != "/orange/laserscan") {
            continue;
        }
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

        serialization.deserialize_message(&serialized_msg, laser_msg.get());
        break;
    }

    ////////////////////////////////////////////
    // Our code is tested below

    // We create an object of the class we are testing
    LaserProcessing laserProcessing(*laser_msg); // We pass the message to the constructor

    // We call the function that we are testing
    unsigned int readings = laserProcessing.countObjectReadings();

    // We check if the function returned the expected value, this should not be a value from your code.
    // rather obtained by other means such as manual calculation or visual inspection of the data
    EXPECT_EQ(readings,640);

}

// You can have multiple tests in a test suite, refer Quiz 5 for examples
