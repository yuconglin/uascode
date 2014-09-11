#pragma once
#include "UserStructs/obstacle3D.h"

#include <fstream>

#include "ros/ros.h"
#include "uascode/WpCurrent.h"
#include "uascode/ObsMsg.h"

namespace UasCode{

class AdsbFromFile{
public:
   AdsbFromFile();
   ~AdsbFromFile();
   //read adsbs
   void ReadADSB(const std::vector<std::string>& file_names);
   //send adsbs from file
   void SendObss2(bool f0, bool f1, bool f2);
   //load offsets from files
   void LoadOffsetsSingle(const char *filename,int idx);
   void LoadOffsets2(const char* off1,const char* off2,const char* off3,const char* type = "");
   void LoadSendConfig(const char *filename);

private:
   struct OffSet{
       double x_off;
       double y_off;
       double z_off;
       double hd_off;

       OffSet():x_off(0),y_off(0),z_off(0),hd_off(0){}
   };

   std::vector<OffSet> offsets;
   std::vector<std::vector<UserStructs::obstacle3D> > all_obss;
   int seq_current;
   //ros related
   ros::NodeHandle nh;
   ros::Publisher pub_obss;
   ros::Subscriber sub_wp_curr;
   //callback functions
   void WpCurrCb(const uascode::WpCurrent::ConstPtr& msg);
   uascode::ObsMsg ObsToRosMsg(const UserStructs::obstacle3D& obs);
};

}
