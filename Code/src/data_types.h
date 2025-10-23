#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include <vector>

namespace data {

    namespace commands {

        struct Quadcopter{
            unsigned long seq;/*!< seq of command, repeated sequence numbers ignored, can be restarted, always starts from 1, seq number equal to zero will be ignored*/
            double turn_l_r;/*!< angular speed of turn, left positive [rad/s] */
            double move_l_r;/*!< speed of left/right motion, left positive [m/s] */
            double move_u_d;/*!< speed of up/down motion, up positive [m/s] */
            double move_f_b;/*!< speed of forward/backward motion, forward positive [m/s] */
            bool takeoff;
        };
        struct SkidSteer{
            unsigned long seq;/*!< seq of command, repeated sequence numbers ignored, can be restarted, always starts from 1, seq number equal to zero will be ignored*/
            double turn_l_r;/*!< angular speed of turn, left positive [rad/s] */
            double move_f_b;/*!< speed of forward/backward motion, forward positive [m/s] */
        };        
    }

    namespace geometry_msgs {
        struct Pose2D{
            double x;/*!< position x [m] */
            double y;/*!< position y [m] */
            double theta;/*!< angle [radians] */
        };

        struct Point{
            double x;/*!< position x [m] */
            double y;/*!< position y [m] */
            double z;/*!< position z[m] */
        };

        struct Vector3{
          double x;/*!< velocity x [m/s] */
          double y;/*!< velocity y [m/s] */
          double z;/*!< velocity z [m/s] */
        };

        struct Quaternion{
          double x; /*!<  x component  */
          double y; /*!<  y component  */
          double z; /*!<  z component  */
          double w; /*!<  w component  */
        };

        struct Goal{
            unsigned long seq;
            Point point;
        };

    }

    namespace nav_msgs{
        struct Odometry{
            double time;/*!< seq of command, repeated sequence numbers ignored, can be restarted, always starts from 1, seq number equal to zero will be ignored*/
            geometry_msgs::Point position; /*!< postion [m] */
            double yaw; /*!< yaw [rad] */
            geometry_msgs::Vector3 linear; /*!< linear velocity [m/s] */
        };
    }

    namespace sensor_msgs{
        struct LaserScan{
            double time;/*!< seq of command, repeated sequence numbers ignored, can be restarted, always starts from 1, seq number equal to zero will be ignored*/
            double angle_min;/*!< angle min (clockwise wrt orientation)  [rad] */
            double angle_max;/*!< angle max (anti-clockwise wrt orientation)  [rad] */
            double angle_increment; /*!< angle increment [rad] */
            double range_min; /*!< minimum range [m] */
            double range_max; /*!< maximum range [m] */
            std::vector<float> ranges; /*!< vector of ranges [m], values < range_min or > range_max should be discarded) */
        };

        struct Sonar{
            double time;/*!< seq of command, repeated sequence numbers ignored, can be restarted, always starts from 1, seq number equal to zero will be ignored*/
            double field_of_view; /*!<  angle increment(centred around the orientation) [rad] */
            double range_min; /*!< minimum range [m] */
            double range_max; /*!< maximum range [m] */
            double range; /*!< range meauremed by sonar [m]  values < range_min or > range_max should be discarded) */
        };
    }

    typedef enum {
      SKIDSTEER, /*!< Skid steer based ground vehicle */
      QUADCOPTER, /*!< Quadcopter */
    } PlatformType; /*!< Platform Types */

    typedef enum {
      IDLE, /*!< Stationary */
      RUNNING, /*!< Executing a motion */
      TAKEOFF, /*!< UAV only, taking off */
      LANDING /*!< UAV only, landing */
    } PlatformStatus; /*!< Platform Status */

    typedef enum {
      POINT, /*!< Point base, like a laser */
      CONE /*!< Cone based like sonar and radar */
    } RangerType; /*!< Ranger Types */

}

#endif // DATA_TYPES_H
