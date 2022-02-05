#include "OccupiedGridObstacleAvoidanceRos/MotionPlanner.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h> 
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/Image.h> 

Planning planning("/home/adachi/ros1_ws/src/OccupiedGridObstacleAvoidanceRos/plot/testcase1.dat");

ros::Publisher path_pub,image_pub;

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

cv::Mat 
getMat(nav_msgs::OccupancyGrid map)
{
  cv::Mat mat(map.info.height, map.info.width, CV_8U, cv::Scalar(0));
  int dc = 0;
  for (int y = 0; y < map.info.height; y++) {
    for (int x = 0; x < map.info.width; x++) {
      mat.at<int8_t>(y, x) = map.data[dc];  // mat
      dc++;
    }
  }
  return mat;
}

void
MapCallback (const nav_msgs::OccupancyGridConstPtr& map_msg){
  std::cout << "get Map" << std::endl;
  planning.cell_size = map_msg->info.resolution;
  planning.map_origin.x() = map_msg->info.origin.position.x;
  planning.map_origin.y() = map_msg->info.origin.position.y;
  double roll,pitch,yaw;
  tf::Quaternion quat;
  quaternionMsgToTF(map_msg->info.origin.orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
  planning.map_origin.z() = yaw;
  planning.obstacle_mat = getMat(*map_msg);

  // std_msgs::Header header;
  // header.frame_id = "map";
  // sensor_msgs::ImagePtr map_image_msg;
  // cv::Mat result;
  // planning.obstacle_mat.convertTo(result, CV_8S);
  // map_image_msg = cv_bridge::CvImage(header, "mono8", result).toImageMsg();
  // image_pub.publish(*map_image_msg);
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