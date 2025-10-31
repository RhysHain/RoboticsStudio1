#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include <vector>

namespace data {
    /// @brief Data structures for storing platform commands
    namespace commands {
        /// @brief Quadcopter Commands
        struct Quadcopter{
            unsigned long seq;/*!< seq of command, repeated sequence numbers ignored, can be restarted, always starts from 1, seq number equal to zero will be ignored*/
            double turn_l_r;/*!< angular speed of turn, left positive [rad/s] */
            double move_l_r;/*!< speed of left/right motion, left positive [m/s] */
            double move_u_d;/*!< speed of up/down motion, up positive [m/s] */
            double move_f_b;/*!< speed of forward/backward motion, forward positive [m/s] */
            bool takeoff;
        };
        /// @brief Skidsteer Commands
        struct SkidSteer{
            unsigned long seq;/*!< seq of command, repeated sequence numbers ignored, can be restarted, always starts from 1, seq number equal to zero will be ignored*/
            double turn_l_r;/*!< angular speed of turn, left positive [rad/s] */
            double move_f_b;/*!< speed of forward/backward motion, forward positive [m/s] */
        };        
    }
    /// @brief Data structures for geometric data types
    namespace geometry_msgs {
        /// @brief 2D Pose
        struct Pose2D{
            double x;/*!< position x [m] */
            double y;/*!< position y [m] */
            double theta;/*!< angle [radians] */
        };
        /// @brief 3D Coordinate
        struct Point{
            double x;/*!< position x [m] */
            double y;/*!< position y [m] */
            double z;/*!< position z[m] */
        };
        /// @brief 3D Vector
        struct Vector3{
          double x;/*!< velocity x [m/s] */
          double y;/*!< velocity y [m/s] */
          double z;/*!< velocity z [m/s] */
        };
        /// @brief Rotational Quaternion
        struct Quaternion{
          double x; /*!<  x component  */
          double y; /*!<  y component  */
          double z; /*!<  z component  */
          double w; /*!<  w component  */
        };
        /// @brief Goal Coordinate
        struct Goal{
            unsigned long seq;
            Point point;
        };

    }
    /// @brief Data structures for navigation purposes
    namespace nav_msgs{
        /// @brief Full odometry of both displcament and velocity
        struct Odometry{
            double time;/*!< seq of command, repeated sequence numbers ignored, can be restarted, always starts from 1, seq number equal to zero will be ignored*/
            geometry_msgs::Point position; /*!< postion [m] */
            double yaw; /*!< yaw [rad] */
            geometry_msgs::Vector3 linear; /*!< linear velocity [m/s] */
        };
    }
    /// @brief Data sturtcures for storing sensor data
    namespace sensor_msgs{
        /// @brief Lidar data
        struct LaserScan{
            double time;/*!< seq of command, repeated sequence numbers ignored, can be restarted, always starts from 1, seq number equal to zero will be ignored*/
            double angle_min;/*!< angle min (clockwise wrt orientation)  [rad] */
            double angle_max;/*!< angle max (anti-clockwise wrt orientation)  [rad] */
            double angle_increment; /*!< angle increment [rad] */
            double range_min; /*!< minimum range [m] */
            double range_max; /*!< maximum range [m] */
            std::vector<float> ranges; /*!< vector of ranges [m], values < range_min or > range_max should be discarded) */
        };
        /// @brief Sonar data
        struct Sonar{
            double time;/*!< seq of command, repeated sequence numbers ignored, can be restarted, always starts from 1, seq number equal to zero will be ignored*/
            double field_of_view; /*!<  angle increment(centred around the orientation) [rad] */
            double range_min; /*!< minimum range [m] */
            double range_max; /*!< maximum range [m] */
            double range; /*!< range meauremed by sonar [m]  values < range_min or > range_max should be discarded) */
        };
    }
    /// @brief Enum for defining platform type
    typedef enum {
      SKIDSTEER, /*!< Skid steer based ground vehicle */
      QUADCOPTER, /*!< Quadcopter */
    } PlatformType; /*!< Platform Types */

    /// @brief Enum for defining platform status
    typedef enum {
      IDLE, /*!< Stationary */
      RUNNING, /*!< Executing a motion */
      TAKEOFF, /*!< UAV only, taking off */
      LANDING /*!< UAV only, landing */
    } PlatformStatus; /*!< Platform Status */

    /// @brief Enum foir defining lidar type
    typedef enum {
      POINT, /*!< Point base, like a laser */
      CONE /*!< Cone based like sonar and radar */
    } RangerType; /*!< Ranger Types */

}

#endif // DATA_TYPES_H
