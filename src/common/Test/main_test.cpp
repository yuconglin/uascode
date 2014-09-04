#include <ros/ros.h>
#include <iostream>
#include "common/Utils/GetTimeNow.h"
#include "common/Utils/GeoUtils.h"

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

  arma::vec::fixed<2> loc;
  arma::vec::fixed<2> pt1;
  arma::vec::fixed<2> pt2;

  loc << 4 << 3;
  pt1 << 0 << 0;
  pt2 << 3 << 0;
  std::cout<<"get_distance:"<< Utils::get_distance(pt1,pt2)<<'\n'
           <<"get_angle:"<< Utils::get_angle(pt2,pt1,loc)*180./M_PI<<'\n'
           <<"location_past_pt:"<< Utils::location_passed_pt(loc,pt1,pt2)<<'\n';

}
