#pragma once
#include "ros/ros.h"

#include "UserStructs/obstacle3D.h"
//ros messages
#include "uascode/WpCurrent.h"
#include "uascode/ObsMsg.h"
#include "yucong_rosmsg/ObsMsg2.h"
#include "std_msgs/UInt16.h"

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
   void SendObss2(bool f1,bool f2,bool f3);
   //set obss_file name
   void SetLogFileName(const char *filename);
   //load offsets from file
   void LoadOffsets(const char *filename);
   void LoadOffsetsSingle(const char *filename,int idx);
   void LoadOffsets2(const char* off1,const char* off2,const char* off3,const char* type = "");
   inline void SetIfMission(const bool _if_mission){this->if_mission = _if_mission;}
   inline void SetIfSendObs(const bool _if_send_obs){this->if_send_obstacle = _if_send_obs;}
   void LoadSendConfig(const char *config_file,const char *obs_file);
   void LoadSendRandom(const char *obs_file,const char* type="");
   void LoadSendRandomNum(const char *obs_file,int num,const char* type="");

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
   bool if_send_obstacle;
   //ros related
   ros::NodeHandle nh;
   ros::Publisher pub_obss;
   ros::Subscriber sub_wp_curr;
   //variables
   std::vector<std::vector<UserStructs::obstacle3D> > all_obss;
   int seq_current;
   //callback functions
   void WpCurrCb(const std_msgs::UInt16::ConstPtr &msg);
   uascode::ObsMsg ObsToRosMsg(const UserStructs::obstacle3D& obs);
   yucong_rosmsg::ObsMsg2 ObsToRosMsg2(const UserStructs::obstacle3D& obs);
   int RandSelectVec(const std::vector<int>& ints);
   std::string int2string(int _num);
   //file for obstacles
   std::ifstream obss_file;
   //file for logging sent obstacles
   std::ofstream obss_log;
};//class ObsFromFile ends

}//namespace ends
