#include "PlanNode.hpp"
#include "ros/ros.h"
using namespace UasCode;

int main(int argc, char** argv)
{
  ros::init(argc,argv,"plan_node");
  PlanNode plan_node;
  plan_node.working();  
}
