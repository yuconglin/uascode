#include "ObsFromFile.hpp"
#include "ros/ros.h"
#include "common/Utils/YcLogger.h"

using namespace UasCode;

int main(int argc,char** argv)
{
   ros::init(argc,argv,"obs_from_file");

   Utils::LogConfigurator myconfigurator("log4cxx.properties", "ObsFromFile");

   ObsFromFile obsfrom;

   if(argc <2)
     obsfrom.SetLogFileName("/home/yucong/ros_workspace/uascode/records/obss_log.txt");
   else if(argc ==2)
     obsfrom.SetLogFileName(argv[1]);
   else {;}

   //obsfrom.ReadSendObss("/home/yucong/ros_workspace/uascode/data/20140516-162158obs.txt",3);
   obsfrom.ReadObss("/home/yucong/ros_workspace/uascode/data/20140516-162158obs.txt");
   //send in ros
   obsfrom.SendObss(3);
}
