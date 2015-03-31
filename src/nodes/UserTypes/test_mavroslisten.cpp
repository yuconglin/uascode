#include "MavrosListen.hpp"
#include "common/Utils/YcLogger.h"
#include "common/Utils/FindPath.h"
#include "ros/ros.h"

using namespace UasCode;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"mavros_listen");
    MavrosListen mav_listen;

    Utils::LogConfigurator myconfigurator("log4cxx_MavrosListen.properties","log for MavrosListen");

    mav_listen.working();
}
