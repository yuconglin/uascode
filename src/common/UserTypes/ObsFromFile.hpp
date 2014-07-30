#pragma once
#include "ros/ros.h"

#include "UserStructs/obstacle3D.h"
//ros messages
#include "uascode/WpCurrent.h"

#include <fstream>

namespace UasCode{

class ObsFromFile{

public:
   ObsFromFile();
   ~ObsFromFile();
   //read obs
   void ReadObss(const char* filename);
   //send obs
   void SendObss(int obs_num);
   //set obss_file name
   void SetLogFileName(const char *filename);
   //load offsets from file
   void LoadOffsets(const char *filename);

private:
   struct OffSet{
       double x_off;
       double y_off;
       double z_off;
       double hd_off;
   };
   std::vector<OffSet> offsets;
   //ros related
   ros::NodeHandle nh;
   ros::Publisher pub_obss;
   ros::Subscriber sub_wp_curr;
   //variables
   std::vector<std::vector<UserStructs::obstacle3D> > all_obss;
   int seq_current;
   //callback functions
   void WpCurrCb(const uascode::WpCurrent::ConstPtr& msg);
   //file for obstacles
   std::ifstream obss_file;
   //file for logging sent obstacles
   std::ofstream obss_log;
};//class ObsFromFile ends

}//namespace ends
