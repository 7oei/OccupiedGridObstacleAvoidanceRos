#include "OccupiedGridObstacleAvoidanceRos/MotionPlanner.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h> 
#include <nav_msgs/OccupancyGrid.h>

Planning planning("/home/adachi/ros1_ws/src/OccupiedGridObstacleAvoidanceRos/plot/testcase1.dat");

ros::Publisher path_pub;
//xStart,yStart,xGoal,yGoal
void
GoalCallback (const geometry_msgs::PoseStampedConstPtr & pose_msg)
{
  // std::cout << "callback start" << std::endl;
  double goal[3];
  goal[0] = pose_msg->pose.position.x;
  goal[1] = pose_msg->pose.position.y;
  goal[2] = 0;
  planning.setGoal(goal);
  std::cout << "goal set" << std::endl;
  nav_msgs::Path planned_path;
  planned_path = planning.planWithSimpleSetup();

  // planning.OpenGnuplot();
  // std::cout << "callback end" << std::endl;
  path_pub.publish(planned_path);
}

void
StartCallback (const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_cov_msg){
  double start[3];
  start[0] = pose_cov_msg->pose.pose.position.x;
  start[1] = pose_cov_msg->pose.pose.position.y;
  start[2] = 0;
  planning.setStart(start);
  std::cout << "start set" << std::endl;
  nav_msgs::Path planned_path;
  planned_path = planning.planWithSimpleSetup();
  path_pub.publish(planned_path);
}

void
MapCallback (const nav_msgs::OccupancyGridConstPtr& map_msg){
  std::cout << "get Map" << std::endl;
}

int
main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "OccupiedGridObstacleAvoidanceRos");
    ros::NodeHandle nh;
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber goal_sub = nh.subscribe ("move_base_simple/goal", 1, GoalCallback);
    ros::Subscriber start_sub = nh.subscribe ("/initialpose", 1, StartCallback);
    ros::Subscriber map_sub = nh.subscribe ("/map", 1, MapCallback);
    // Create a ROS publisher for the output point cloud
    // vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/ndt_ellipsoid", 10);
    // vis_pub2 = nh.advertise<visualization_msgs::MarkerArray>("/neighbor_ellipsoid", 10);
    path_pub = nh.advertise<nav_msgs::Path>("planned_path", 1000);
    // Spin
    ros::spin ();
}