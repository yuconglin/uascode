#include "ScaleObsFromFile.hpp"
#include "ros/ros.h"
#include "common/Utils/YcLogger.h"
#include "common/Utils/FindPath.h"
#include <string>
#include <iostream>

using namespace UasCode;

int main( int argc, char** argv )
{
    ros::init( argc, argv, "scaled_obs_file" );
    Utils::LogConfigurator myconfigurator("log4cxx_ObsFromFile.properties", "ScaleObsFromFile");

    std::string data_file = Utils::FindPath() + argv[1];

    ScaleObsFromFile sof;
    sof.SetIfMission(false);
    sof.LoadSendConfig( argv[2], data_file.c_str() );
}
