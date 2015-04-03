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

    Utils::LogConfigurator myconfigurator("log4cxx_MavrosListen.properties","log for MavrosListen");

    mav_listen.working();
}
