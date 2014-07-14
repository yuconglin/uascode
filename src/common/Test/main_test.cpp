#include <ros/ros.h>
#include <iostream>
#include "common/Utils/GetTimeNow.h"

int main(int argc,char** argv)
{
  //double secs =ros::Time::now().toSec();
  ros::Duration d(0.5);
  double secs = d.toSec();
  std::cout<<"secs: "<< secs<< std::endl;

  ros::Time::init();
  ros::Time t1= ros::Time::now();
  double dt1= Utils::GetTimeNow(); 
  while(1){
   //if(ros::Time::now()-t1 > ros::Duration(1))
   if(ros::Time::now().toSec()-t1.toSec()> 1.0)
     break;
  }//while ends
  std::cout<<"t_diff: "
      << Utils::GetTimeNow()- dt1<< std::endl;
}
