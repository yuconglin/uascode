#include "PlanNode.hpp"
#include "common/Utils/YcLogger.h"
#include "ros/ros.h"
#include <iostream>

using namespace UasCode;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"predict_node");
    PlanNode plan_node;

    Utils::LogConfigurator myconfigurator("log4cxx_PlanNode.properties", "log for PlanNode");

    plan_node.SetHomeAlt(585);
    plan_node.LoadFlightPlan(argv[1]);

    if (argc == 2)
       plan_node.SetLogFileName("/home/yucong/ros_workspace/uascode/records/traj_log.txt");
    else if(argc==3)
       plan_node.SetLogFileName(argv[2]);
    else
       std::cerr <<"number of arguments should be only 2 or 3." <<"/n";

    plan_node.predicting();
}
