#include "ObsFromFile.hpp"
#include "ros/ros.h"

using namespace UasCode;

int main(int argc,char** argv)
{
   ros::init(argc,argv,"obs_from_file");
   ObsFromFile obsfrom;
   obsfrom.ReadSendObss("/home/yucong/ros_workspace/uascode/data/20140516-162158obs.txt",3);
}
