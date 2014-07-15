#pragma once
#include "ros/ros.h"

namespace UasCode{

class ObsFromFile{

public:
   ObsFromFile();
   ~ObsFromFile();
   //read obs
   void ReadSendObss(const char* filename,int obs_num);
   inline void SetMaxObsNum(const int _num){num_max= _num;}
private:
   ros::NodeHandle nh;
   ros::Publisher pub_obss;
   int num_max;

};//class ObsFromFile ends

}//namespace ends
