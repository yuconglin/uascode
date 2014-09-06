#pragma once
#include "ros/ros.h"

#include "UserStructs/obstacle3D.h"
//ros messages
#include "uascode/WpCurrent.h"

#include <fstream>
#include <map>

namespace UasCode{

class ObsFromFile{

public:
   ObsFromFile();
   ~ObsFromFile();
   //read obs
   void ReadObss(const char* filename);
   //send obs
   void SendObss(int obs_num);
   void SendObss2(bool f1,bool f2,bool f3);
   //set obss_file name
   void SetLogFileName(const char *filename);
   //load offsets from file
   void LoadOffsets(const char *filename);
   void LoadOffsetsSingle(const char *filename,int idx);
   void LoadOffsets2(const char* off1,const char* off2,const char* off3);
   void SetIfMission(const bool _if_mission){this->if_mission = _if_mission;}

private:
   struct OffSet{
       double x_off;
       double y_off;
       double z_off;
       double hd_off;

       OffSet():x_off(0),y_off(0),z_off(0),hd_off(0){}
   };
   std::map<uint32_t,int> addrs_map;
   std::vector<OffSet> offsets;
   bool if_mission;
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
