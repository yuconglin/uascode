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
    std::string file_adsb0 = "tt_obstacle_10734770.txt";
    std::string file_adsb1 = "tt_obstacle_10934723.txt";
    std::string file_adsb2 = "tt_obstacle_10942331.txt";

    std::vector<std::string> file_names;
    file_names.push_back(file_adsb0);
    file_names.push_back(file_adsb1);
    file_names.push_back(file_adsb2);

    AdsbFromFile adsbfrom;

    //adsbfrom.ReadADSB(file_names);
    //adsbfrom.LoadOffsets2("20","20","20","A");
    //adsbfrom.SendObss2(false,true,false);
    //adsbfrom.LoadSendConfig(argv[1],file_names);
    adsbfrom.LoadSendRandom(file_names,"A");

    return 0;
}
