#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <rosgraph_msgs/Clock.h>
#include <dynamic_reconfigure/server.h>
#include <controllers/setTrajectoryConfig.h>
#include <Eigen/Dense>
#include <controllers/FlatTarget.h>
#include <nav_msgs/Odometry.h>

using namespace geometry_msgs;
using namespace std;
using namespace ros;
using namespace Eigen;

using Eigen::Vector4d;

class Trajectory{
  public:
    Trajectory(int, char**);
    ~Trajectory();
    void dynamicReconfigureCallback(controllers::setTrajectoryConfig &config, uint32_t level);
    void cmdloopCallback(const ros::TimerEvent& event);
  private:
    
    // loop timer sibscriber

    ros::Timer cmdloop_timer_;
    // Publishers
    
    ros::Publisher trajectory_publisher;
    ros::Publisher velocity_publisher;
    ros::Publisher odom_publisher_;
    ros::Publisher flat_publisher_;

    ros::Publisher pos_publisher_;
    ros::Publisher cmdvel_publisher_;

    ros::Time time_now_, time_previous_;
    
    double distance(Vector4d v1, Vector4d v2);
    // Pose
    Vector4d pose_d;
    // Trajectory speed
    double speed;
    // Trajectory type
    int trajectory_type;
    // Waypoints
    MatrixXd waypoints;
    int waypoint;

    // Time
    double t;
};
