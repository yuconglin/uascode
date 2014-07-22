#pragma once
#include "ros/ros.h"

#include "UserStructs/obstacle3D.h"
//ros messages
#include "uascode/WpCurrent.h"

namespace UasCode{

class ObsFromFile{

public:
   ObsFromFile();
   ~ObsFromFile();
   //read obs
   void ReadObss(const char* filename);
   //send obs
   void SendObss(int obs_num);
   //read while send
   void ReadSendObss(const char *filename,int obs_num);

private:
   //ros related
   ros::NodeHandle nh;
   ros::Publisher pub_obss;
   ros::Subscriber sub_wp_curr;
   //variables
   std::vector<std::vector<UserStructs::obstacle3D> > all_obss;
   int seq_current;
   //callback functions
   void WpCurrCb(const uascode::WpCurrent::ConstPtr& msg);

};//class ObsFromFile ends

}//namespace ends
