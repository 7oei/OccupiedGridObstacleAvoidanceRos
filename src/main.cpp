#include "OccupiedGridObstacleAvoidanceRos/MotionPlanner.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

void
PoseCallback (const geometry_msgs::PoseStampedConstPtr & pose_msg)
{
  std::cout << "callback start" << std::endl;
  Planning planning("/home/adachi/ros1_ws/src/OccupiedGridObstacleAvoidanceRos/plot/testcase1.dat");
  planning.planWithSimpleSetup();
  // planning.OpenGnuplot();
  std::cout << "callback end" << std::endl;
}

int
main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "OccupiedGridObstacleAvoidanceRos");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber pose_sub = nh.subscribe ("move_base_simple/goal", 1, PoseCallback);

    // Create a ROS publisher for the output point cloud
    // vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/ndt_ellipsoid", 10);
    // vis_pub2 = nh.advertise<visualization_msgs::MarkerArray>("/neighbor_ellipsoid", 10);
    // Spin
    ros::spin ();
}