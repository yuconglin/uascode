#include "AdsbFromFile.hpp"
#include "ros/ros.h"
#include "common/Utils/YcLogger.h"
#include "common/Utils/FindPath.h"
#include <string>

using namespace UasCode;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"adsb_from_file");
    Utils::LogConfigurator myconfigurator("log4cxx_AdsbFromFile.properties","AdsbFromFile");

    //rank from small to large
    std::string file_adsb0 = "whole_obstacle_10734770.txt";
    //std::string file_adsb1 = "whole_obstacle_10934723.txt";
    std::string file_adsb1 = "obstacle_10934723.txt";
    std::string file_adsb2 = "whole_obstacle_10942331.txt";

    std::vector<std::string> file_names;
    file_names.push_back(file_adsb0);
    file_names.push_back(file_adsb1);
    file_names.push_back(file_adsb2);

    AdsbFromFile adsbfrom;
    adsbfrom.LoadOffsets2("10","10","10","A");
    adsbfrom.ReadADSB(file_names);
    adsbfrom.SendObss2(false,true,false);

    return 0;
}
