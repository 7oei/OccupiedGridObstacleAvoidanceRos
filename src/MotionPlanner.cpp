#include "OccupiedGridObstacleAvoidanceRos/MotionPlanner.h"
#include "OccupiedGridObstacleAvoidanceRos/OptimizationObjectives.h"
using namespace std;
// コンストラクタ
Planning::Planning(std::string fileName){
  initFromFile(fileName);
  CreateCube();
  // PlannerSelector();
  // T-RRT固定
  selector = 7;
}

void Planning::setStart(double start[3]){
  xStart = start[0];
  yStart = start[1];
}

void Planning::setGoal(double goal[3]){
  xGoal = goal[0];
  yGoal = goal[1];
}

void Planning::initFromFile(std::string fileName)
{
  std::ifstream input(fileName.c_str());

  input >> xLeft >> xRight >> yBottom >> yTop >> numObstacles;

  xMin = new double[numObstacles];
  xMax = new double[numObstacles];
  yMin = new double[numObstacles];
  yMax = new double[numObstacles];

  for (int i = 0; i < numObstacles; ++i){
    input >> xMin[i] >> xMax[i] >> yMin[i] >> yMax[i];
  }

  input >> xStart >> yStart >> xGoal >> yGoal >> stepSize;

  input.close();

  printf("\nフィールドの定義域は: x[%5.2lf, %5.2lf] y[%5.2lf, %5.2lf]\n", xLeft, xRight, yBottom, yTop);

  cout << "障害物リスト" << endl;
  for (int i = 0; i < numObstacles; ++i){
    printf("             障害物%d: x[%5.2lf, %5.2lf] y[%5.2lf, %5.2lf]\n", i+1, xMin[i], xMax[i], yMin[i], yMax[i]);
  }

  printf("\nスタートとゴール    : Start[%5.2lf, %5.2lf]\n", xStart, yStart);
  printf("                        End[%5.2lf, %5.2lf]\n\n", xGoal, yGoal);
}


void Planning::CreateCube()
{
  RANGE obstacle;
  ofstream cube("/home/adachi/ros1_ws/src/OccupiedGridObstacleAvoidanceRos/plot/obstacle.dat");
  ofstream plot_start("/home/adachi/ros1_ws/src/OccupiedGridObstacleAvoidanceRos/plot/start.dat");
  ofstream plot_goal("/home/adachi/ros1_ws/src/OccupiedGridObstacleAvoidanceRos/plot/goal.dat");

  plot_start << xStart << "\t" << yStart << std::endl;
  plot_goal << xGoal << "\t" << yGoal << std::endl;

  for(int ob = 0; ob < numObstacles; ++ob){
    obstacle.xrange[0] = xMin[ob]; obstacle.yrange[0] = yMin[ob];
    obstacle.xrange[1] = xMax[ob]; obstacle.yrange[1] = yMax[ob];
    for (int i = 0; i < 2; ++i){
      cube << obstacle.xrange[0] << "\t" << obstacle.yrange[0] << "\t" << std::endl;//ポリゴン辿ってる
      cube << obstacle.xrange[1] << "\t" << obstacle.yrange[0] << "\t" << std::endl;
      cube << obstacle.xrange[1] << "\t" << obstacle.yrange[1] << "\t" << std::endl;
      cube << obstacle.xrange[0] << "\t" << obstacle.yrange[1] << "\t" << std::endl;
      cube << obstacle.xrange[0] << "\t" << obstacle.yrange[0] << "\t" << std::endl;
      cube << "\n\n";
    }
  }
}


void Planning::PlannerSelector()
{
  std::string plan[9] = {"PRM",    "RRT",     "RRTConnect", "RRTstar",
                         "LBTRRT", "LazyRRT", "TRRT",       "pRRT",
                         "EST"};
  std::string yn;
  while (1) {
    cout << "プランナーを選択してください" << endl;

    printf("PRM        → 1\n");
    printf("RRT        → 2\n");
    printf("RRTConnect → 3\n");
    printf("RRTstar    → 4\n");
    printf("LBTRRT     → 5\n");
    printf("LazyRRT    → 6\n");
    printf("TRRT       → 7\n");
    printf("pRRT       → 8\n");
    printf("EST        → 9\n");

    while (1) {
      cout << "数字を入力 >>";
      cin >> selector;
      if(1 <= selector && selector <= 9){
        break;
      }
    }

    cout << plan[selector - 1] << "プランナーを使います よろしいですか？(y/n)" << endl;
    cin >> yn;
    if(yn == "y"){
      break;
    }
  }
}


bool Planning::isStateValid(const ob::State *state)
{
  const ob::SE2StateSpace::StateType *state_2d= state->as<ob::SE2StateSpace::StateType>();
  const double &x(state_2d->getX()), &y(state_2d->getY());

  for (int i = 0; i < numObstacles; ++i){
    if (xMin[i] <= x && x <= xMax[i] && yMin[i] <= y && y <= yMax[i]){
      return false;
    }
  }

  return true;
}


// Print a vertex to file
void Planning::printEdge(std::ostream &os, const ob::StateSpacePtr &space, const ob::PlannerDataVertex &vertex)
{
  std::vector<double> reals;
  if(vertex!=ob::PlannerData::NO_VERTEX)// 頂点が存在しない状態じゃなかったら
  {
    space->copyToReals(reals, vertex.getState());// Copy all the real values from a state source to the array reals using getValueAddressAtLocation()
    for(size_t j = 0; j < reals.size(); ++j){
      os << " " << reals[j];
    }
  }
}

nav_msgs::Path Planning::extractPath(ob::ProblemDefinition* pdef){
    nav_msgs::Path plannedPath;
    plannedPath.header.frame_id = "/map";
    // get the obtained path
    cout<<"get path in extractPath start"<<endl;
    ob::PathPtr path = pdef->getSolutionPath();
    cout<<"get path in extractPath end"<<endl;
    // convert to geometric path
    cout<<"get geometric start"<<endl;
    const auto *path_ = path.get()->as<og::PathGeometric>();
    cout<<"get geometric end"<<endl;
    // iterate over each position
    for(unsigned int i=0; i<path_->getStateCount(); ++i){
        // get state
        const ob::State* state = path_->getState(i);
        // // get x coord of the robot
        // const auto *coordX =
        //         state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
        // // get y coord of the robot
        // const auto *coordY =
        //         state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
        const ob::SE2StateSpace::StateType *state_2d= state->as<ob::SE2StateSpace::StateType>();
        const double &x(state_2d->getX()), &y(state_2d->getY());
        // fill in the ROS PoseStamped structure...
        geometry_msgs::PoseStamped poseMsg;
        poseMsg.pose.position.x = x;
        poseMsg.pose.position.y = y;
        poseMsg.pose.position.z = 0.01;
        poseMsg.pose.orientation.w = 1.0;
        poseMsg.pose.orientation.x = 0.0;
        poseMsg.pose.orientation.y = 0.0;
        poseMsg.pose.orientation.z = 0.0;
        poseMsg.header.frame_id = "/map";
        poseMsg.header.stamp = ros::Time::now();
        // ... and add the pose to the path
        plannedPath.poses.push_back(poseMsg);
    }
    return plannedPath;
}


nav_msgs::Path Planning::planWithSimpleSetup()
{
  // Construct the state space where we are planning
  ob::StateSpacePtr space(new ob::SE2StateSpace());

  ob::RealVectorBounds bounds(2);
  bounds.setLow(0,xLeft);
  bounds.setHigh(0,xRight);
  bounds.setLow(1,yBottom);
  bounds.setHigh(1,yTop);
  space->as<ob::SE2StateSpace>()->setBounds(bounds);

  ob::SpaceInformationPtr si(new ob::SpaceInformation(space));

  // Instantiate SimpleSetup
  og::SimpleSetup ss(space);

  // Setup the StateValidityChecker
  ss.setStateValidityChecker(boost::bind(&Planning::isStateValid, this, _1));

  // Setup Start and Goal
  ob::ScopedState<ob::SE2StateSpace> start(space);
  start->setXY(xStart,yStart);
  cout << "start: "; start.print(cout);

  ob::ScopedState<ob::SE2StateSpace> goal(space);
  goal->setXY(xGoal,yGoal);
  cout << "goal: "; goal.print(cout);

  ss.setStartAndGoalStates(start, goal);

  if(selector == 1){
    ob::PlannerPtr planner(new og::PRM(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }else if(selector == 2){
    ob::PlannerPtr planner(new og::RRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }else if(selector == 3){
    ob::PlannerPtr planner(new og::RRTConnect(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }else if(selector == 4){
    ob::PlannerPtr planner(new og::RRTstar(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }else if(selector == 5){
    ob::PlannerPtr planner(new og::LBTRRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }else if(selector == 6){
    ob::PlannerPtr planner(new og::LazyRRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }else if(selector == 7){
    ob::PlannerPtr planner(new og::TRRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }else if(selector == 8){
    ob::PlannerPtr planner(new og::pRRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }else if(selector == 9){
    ob::PlannerPtr planner(new og::EST(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }

  cout << "----------------" << endl;

  ompl::base::OptimizationObjectivePtr objective;
  objective.reset(new ob::MechanicalWorkOptimizationObjectiveMod(si, 0.1));
  // objective.reset(new ompl::base::PathLengthOptimizationObjective(si));
  ss.setOptimizationObjective(objective);

  // Execute the planning algorithm
  ob::PlannerStatus solved = ss.solve(10.0);

  // If we have a solution,ClearanceObjective
  if (solved)
  {
    // Print the solution path (that is not simplified yet) to a file
    std::ofstream ofs0("/home/adachi/ros1_ws/src/OccupiedGridObstacleAvoidanceRos/plot/path0.dat");//ギザギザ経路
    ss.getSolutionPath().printAsMatrix(ofs0);

    // Simplify the solution        //ここで経路のエッジを減らしてる！！
    ss.simplifySolution();
    cout << "----------------" << endl;
    cout << "Found solution:" << endl;
    // Print the solution path to screen
    ss.getSolutionPath().print(cout);
    // Print the solution path to a file
    std::ofstream ofs("/home/adachi/ros1_ws/src/OccupiedGridObstacleAvoidanceRos/plot/path.dat");
    ss.getSolutionPath().printAsMatrix(ofs);
    //経路を取得
    // auto pdef(std::make_shared<ob::ProblemDefinition>(ss));
    cout << "get path start" << endl;
    auto pdef = ss.getProblemDefinition();
    nav_msgs::Path planned_path;
    planned_path = extractPath(pdef.get());
    cout << "get path end" << endl;

    // Get the planner data to visualize the vertices and the edges
    ob::PlannerData pdat(ss.getSpaceInformation());
    ss.getPlannerData(pdat);

    // Print the vertices to file
    std::ofstream ofs_v("/home/adachi/ros1_ws/src/OccupiedGridObstacleAvoidanceRos/plot/vertices.dat");
    for(unsigned int i(0); i<pdat.numVertices(); ++i)
    {
      printEdge(ofs_v, ss.getStateSpace(), pdat.getVertex(i));
      ofs_v<<endl;
    }

    // Print the edges to file
    std::ofstream ofs_e("/home/adachi/ros1_ws/src/OccupiedGridObstacleAvoidanceRos/plot/edges.dat");
    std::vector<unsigned int> edge_list;
    for(unsigned int i(0); i<pdat.numVertices(); ++i)
    {
      unsigned int n_edge= pdat.getEdges(i,edge_list);
      for(unsigned int i2(0); i2<n_edge; ++i2)
      {
        printEdge(ofs_e, ss.getStateSpace(), pdat.getVertex(i));
        ofs_e<<endl;
        printEdge(ofs_e, ss.getStateSpace(), pdat.getVertex(edge_list[i2]));
        ofs_e<<endl;
        ofs_e<<endl<<endl;
      }
    }
    return planned_path;
  }
  else
    cout << "No solution found" << endl;
}


void Planning::output_plt(std::string plt_output)
{
  std::ofstream plt(plt_output);

  plt << "#set terminal postscript eps color enhanced 20" << endl;
  plt << "#set output \"out.eps\"" << endl;

  plt << "set xlabel \"x\""<< endl;
  plt << "set ylabel \"y\"" << endl;
  plt << "set xrange [" << xLeft << ":" << xRight << "]" << endl;
  plt << "set yrange [" << yBottom << ":" << yTop << "]" << endl;
  plt << "set key outside" << endl;
  plt << "set key top right" << endl;
  plt << "set size square" << endl;

  plt << "plot \"obstacle.dat\" using 1:2 with filledcurves lt rgb \"#ff0033\" fill solid 0.5 notitle,\\" << endl;
  plt << "\"start.dat\" using 1:2 with points pt 7 ps 1.5 lt rgb \"#ff9900\" title \"Start\",\\" << endl;
  plt << "\"goal.dat\" using 1:2 with points pt 7 ps 1.5 lt rgb \"#15BB15\" title \"Goal\",\\" << endl;
  plt << "\"edges.dat\" using 1:2 with lines lt 1 lc rgb \"#728470\" lw 0.5 title \"edges\",\\" << endl;
  if(selector == 1){
    plt << "\"vertices.dat\" using 1:2 with points pt 7 ps 1 lt rgb \"#5BBC77\" title \"Vertices\",\\" << endl;
  }
  plt << "\"path.dat\" using 1:2 with lines lt 1 lc rgb \"#191970\" lw 2 title \"Path\",\\" << endl;
  plt << "\"path0.dat\" using 1:2 with lines lt 1 lc rgb \"#ff4500\" lw 2 title \"Path0\"" << endl;
}


// 参考：http://www-sens.sys.es.osaka-u.ac.jp/wakate/tutorial/group3/gnuplot/
int Planning::OpenGnuplot()
{
  output_plt("/home/adachi/ros1_ws/src/OccupiedGridObstacleAvoidanceRos/plot/plot.plt");

  FILE *fp = popen("cd /home/adachi/ros1_ws/src/OccupiedGridObstacleAvoidanceRos/plot && gnuplot -persist", "w");
  if (fp == NULL){
    return -1;
  }
  fputs("set mouse\n", fp);
  fputs("load \"plot.plt\"\n", fp);

  fflush(fp);
  cin.get();
  pclose(fp);
  return 0;
}