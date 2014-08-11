#include "MavlinkSendNode.hpp"
#include "common/Utils/YcLogger.h"

using namespace UasCode;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"MavSendNode");

    Utils::LogConfigurator myconfigurator("log4cxx_MavSendNode.properties", "log for MavSendNode");

    MavlinkSendNode mavsend_node;
    mavsend_node.SetSendPosMethod(atof(argv[1]));
    mavsend_node.PortSetUp();
    mavsend_node.working();
}
