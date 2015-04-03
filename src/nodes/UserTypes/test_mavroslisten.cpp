#include "MavrosListen.hpp"
#include "common/Utils/YcLogger.h"
#include "common/Utils/FindPath.h"
#include "ros/ros.h"

#include <cstdlib>

using namespace UasCode;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"mavros_listen");
    MavrosListen mav_listen;

    Utils::LogConfigurator myconfigurator("log4cxx_MavrosListen.properties","log for MavrosListen");

    if( argc > 1 ){
      mav_listen.SetWpR(atof(argv[1]));
    }
    else{
      mav_listen.SetWpR(25);
    }

    if( argc > 2 ){
      mav_listen.SetHomeAlt( atof(argv[2]) );
    }
    else{
      mav_listen.SetHomeAlt(690.15);
    }

    std::string traj_file = Utils::FindPath()+"recordsHIL/traj_log.txt";
    mav_listen.SetLogFileName(traj_file.c_str());

    mav_listen.working();
}
