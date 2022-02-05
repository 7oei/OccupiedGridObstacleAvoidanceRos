#ifndef _MOTION_PLANNER_H_
#define _MOTION_PLANNER_H_
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/est/EST.h>

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/PlannerData.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <ostream>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <boost/bind.hpp>

#include <ros/ros.h>
#include <nav_msgs/Path.h> 
#include <nav_msgs/OccupancyGrid.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

typedef struct {
  double xrange[2];
  double yrange[2];
} RANGE;



class Planning{
  public:
    Planning(std::string fileName);
    void initFromFile(std::string fileName);
    void setGoal(double goal[3]);
    void setStart(double start[3]);
    void CreateCube();
    void PlannerSelector();
    void printEdge(std::ostream &os, const ob::StateSpacePtr &space, const ob::PlannerDataVertex &vertex);
    nav_msgs::Path extractPath(ob::ProblemDefinition* pdef);
    bool isStateValid(const ob::State *state);
    nav_msgs::Path planWithSimpleSetup();
    void output_plt(std::string plt_output);
    int OpenGnuplot();
    Eigen::Vector2i m2pix(Eigen::Vector2d m_point, float cell_size, Eigen::Vector3d origin);
    Eigen::Vector3d map_origin;
    float cell_size = 0.05; 
    cv::Mat obstacle_mat;
  private:
    double* xMin;
    double* xMax;
    double* yMin;
    double* yMax;
    // Number of obstacles in space.
    int numObstacles;
    // Start position in space
    double xStart;
    double yStart;
    // Goal position in space
    double xGoal;
    double yGoal;
    // Max. distance toward each sampled position we
    // should grow our tree
    double stepSize;
    // Boundaries of the space
    double xLeft;
    double xRight;
    double yTop;
    double yBottom;

    int selector;
};
#endif