#pragma once
#include "ros/ros.h"

#include "UserStructs/obstacle3D.h"
//ros message
#include "uascode/WpCurrent.h"
#include <fstream>

namespace UasCode{

class AdsbFromFile{
public:
    AdsbFromFile();
    ~AdsbFromFile();
    //read obs
    void ReadADSBs(const std::vector<string> file_names);
    //send obs
    void SendObss2(bool f1,bool f2,bool f3);
    //load offsets from file
    void LoadOffsets2(const char* off1,const char* off2,const char* off3,const char* type = " ");

private:
    struct OffSet{
        double x_off;
        double y_off;
        double z_off;
        double hd_off;
        OffSet():x_off(0.),y_off(0.),z_off(0.),hd_off(0.){}
    };

    //variables
    std::vector<OffSet> offsets;
    std::vector<std::vector<UserStructs::obstacle3D> > all_obss;
    int seq_current;

    //ros related
    ros::NodeHandle nh;
    ros::Publisher pub_obss;
    ros::Subscriber sub_wp_curr;

};

}
