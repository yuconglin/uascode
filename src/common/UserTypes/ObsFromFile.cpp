#include "ObsFromFile.hpp"
#include "fstream"
#include <vector>
#include "Planner/UserStructs/obstacle3D.h"
#include "common/Utils/GetTimeUTC.h"
#include "common/Utils/GetTimeNow.h"
//ros
#include "uascode/MultiObsMsg.h"

namespace UasCode{

 //free function
 uascode::ObsMsg ObsToRosMsg(const UserStructs::obstacle3D& obs);
 //free function ends

 ObsFromFile::ObsFromFile():seq_current(-1)
 {
   pub_obss=nh.advertise<uascode::MultiObsMsg>("multi_obstacles",1);
   sub_wp_curr = nh.subscribe("waypoint_current",100,&ObsFromFile::WpCurrCb,this);
 }

 ObsFromFile::~ObsFromFile(){}

 void ObsFromFile::SetLogFileName(const char *filename)
 {
   try
   {
     obss_log.open(filename,std::ofstream::out
                    | std::ofstream::in
                    | std::ofstream::trunc);
   }
   catch (std::ofstream::failure& e) {
         std::cerr << "Exception opening/reading file "
                   << e.what()
                   << std::endl;
     }
}

 void ObsFromFile::ReadObss(const char *filename)
 {
     std::vector<UserStructs::obstacle3D> obss;
     UserStructs::obstacle3D obs_single;

     std::cout<<"ReadObss starts" << std::endl;

     try
     {
       obss_file.open(filename);
     }
     catch (std::ifstream::failure& e) {
           std::cerr << "Exception opening/reading file "
                     << e.what()
                     << std::endl;
     }

     while(obss_file.good() )
     {
         bool repeat= false;

         obss_file >> obs_single.address
                   >> obs_single.x1
                 >> obs_single.x2
                 >> obs_single.x3
                 >> obs_single.head_xy
                 >> obs_single.speed
                 >> obs_single.v_vert
                 >> obs_single.t
                 >> obs_single.r
                 >> obs_single.hr;

         obs_single.head_xy= obs_single.head_xy/M_PI*180.;

         for(int i=0;i!= obss.size();++i)
         {
             if(obs_single.address== obss[i].address)
                 repeat= true;
             break;
         }

         if(repeat){
            all_obss.push_back(obss);
            obss.clear();
         }

         //push
         obss.push_back(obs_single);
     }//while ends
     std::cout<<"ReadObss endss" << "\n";
     std::cout<<"all_obss size: " << all_obss.size() << "\n";

 }

 void ObsFromFile::SendObss(int obs_num)
 {
     uascode::MultiObsMsg obss_msg;
     //please pay attention to time of obstacles
     //being sent

     //index of obstacles
     int i= 0;
     ros::Rate r(10);

     while(ros::ok() )
     {
        ros::spinOnce();

        if(seq_current > 0 && i!= all_obss.size() )
        {//start to publish obstacles
          std::cout<< "send obstacles." << std::endl;

          std::vector<UserStructs::obstacle3D> obss
                  = all_obss[i];
          obss_msg.MultiObs.clear();

          int up_limit= obs_num > obss.size() ? obss.size():obs_num;

          for(int i=0;i!= up_limit;++i){
              //here obs_num is used to select num of obstacles to use
              //otherwise we can just use obss.size()
              //obss[i].t= Utils::GetTimeUTC();
              obss[i].t= Utils::GetTimeNow();

              //log to file
              if(obss_log.is_open() )
              {
                  std::cout<< "opens ok" << std::endl;
                  obss_log << obss[i].address << " "
                           << obss[i].x1 << " "
                           << obss[i].x2 << " "
                           << obss[i].x3 << " "
                           << obss[i].head_xy  << " "
                           << obss[i].speed << " "
                           << obss[i].v_vert << " "
                           << std::setprecision(4) << std::fixed
                           << obss[i].t << " "
                           << obss[i].r << " "
                           << obss[i].hr
                           << "\n";
              }
              obss_msg.MultiObs.push_back(ObsToRosMsg(obss[i]));
          }
          //ros publish
          pub_obss.publish(obss_msg);
          ++i;
          sleep(1);
        }

        r.sleep();

     }//while ends

 }

 void ObsFromFile::WpCurrCb(const uascode::WpCurrent::ConstPtr &msg)
 {
    seq_current= msg->wp_current;
    /*
    std::cout<<"current waypoint #: "
             << seq_current
             << std::endl;
    */
 }

 uascode::ObsMsg ObsToRosMsg(const UserStructs::obstacle3D& obs)
 {
  uascode::ObsMsg obs_msg;
  obs_msg.address= obs.address;
  obs_msg.x1= obs.x1;
  obs_msg.x2= obs.x2;
  obs_msg.x3= obs.x3;
  obs_msg.head_xy= obs.head_xy;
  obs_msg.speed= obs.speed;
  obs_msg.v_vert= obs.v_vert;
  obs_msg.t= obs.t;
  obs_msg.r= obs.r;
  obs_msg.hr= obs.hr;
  return obs_msg;
 }

}//namespace ends
