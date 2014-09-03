#include "PlanNode2.hpp"
#include "common/Utils/YcLogger.h"
#include "common/Utils/FindPath.h"
#include "ros/ros.h"
#include <iostream>

using namespace UasCode;

int main(int argc, char** argv)
{
  ros::init(argc,argv,"plan_node");
  PlanNode2 plan_node;

  Utils::LogConfigurator myconfigurator("log4cxx_PlanNode.properties", "log for PlanNode");

  plan_node.SetHomeAlt(585);
  plan_node.LoadFlightPlan(argv[1]);

  if (argc == 2){
     std::string traj_file = Utils::FindPath()+"records/traj_log.txt";
     plan_node.SetLogFileName(traj_file.c_str());
  }
  else if(argc==3)
     plan_node.SetLogFileName(argv[2]);
  else
     std::cerr <<"number of arguments should be only 2 or 3." <<"/n";

  std::string obdis_file = Utils::FindPath()+"records/obdis_log.txt";
  plan_node.SetObsDisFile(obdis_file.c_str());
  plan_node.working();
}

