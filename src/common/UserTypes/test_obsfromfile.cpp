#include "ObsFromFile.hpp"
#include "ros/ros.h"
#include "common/Utils/YcLogger.h"
#include "common/Utils/FindPath.h"
#include <string>
#include <iostream>

using namespace UasCode;

int main(int argc,char** argv)
{
   ros::init(argc,argv,"obs_from_file");
   std::cout<<"usage: ./test_obsfromfile obss_log_file offsets_file data_file"<< "\n";

   Utils::LogConfigurator myconfigurator("log4cxx_ObsFromFile.properties", "ObsFromFile");

   std::string filepath = Utils::FindPath() ;
   std::string log_file= filepath + argv[1];
   //std::string offsets_file= filepath + argv[2];
   std::string data_file= filepath + argv[2];

   ObsFromFile obsfrom;
   obsfrom.SetIfMission(true);
   obsfrom.SetIfSendObs(true);

   obsfrom.SetLogFileName(log_file.c_str());

   //obsfrom.LoadSendConfig(argv[3],data_file.c_str());
   obsfrom.LoadSendRandom(data_file.c_str(),"A");
}
