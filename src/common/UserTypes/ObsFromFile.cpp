#include "ObsFromFile.hpp"
#include "fstream"
#include <vector>
#include "Planner/UserStructs/obstacle3D.h"
#include "common/Utils/GetTimeUTC.h"
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

 void ObsFromFile::ReadObss(const char *filename)
 {
     std::vector<UserStructs::obstacle3D> obss;
     UserStructs::obstacle3D obs_single;

     std::ifstream obss_file(filename, std::ifstream::in);

     std::cout<<"ReadObss starts" << std::endl;

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
     std::cout<<"ReadObss endss" << std::endl;
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
              obss[i].t= Utils::GetTimeUTC();
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

 void ObsFromFile::ReadSendObss(const char *filename,int obs_num)
 {
   //now we are going to seperate it to read (not in ros)
   //and send in ros
   uascode::MultiObsMsg obss_msg;
   std::vector<UserStructs::obstacle3D> obss;
   UserStructs::obstacle3D obs_single;

   std::ifstream obss_file(filename, std::ifstream::in);

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
     /*
     if(!if_start) t0= obs_single.t;
     obs_single.t -= t0;
     */
     obs_single.t= Utils::GetTimeUTC();
     obs_single.head_xy= obs_single.head_xy/M_PI*180.;

     for(int i=0;i!= obss.size();++i)
     {
         if(obs_single.address== obss[i].address)
             repeat= true;
         break;
     }

     if(repeat){
        for(int i=0;i!= obs_num;++i){
          //here obs_num is used to select num of obstacles to use
          //otherwise we can just use obss.size()
          obss_msg.MultiObs.push_back(ObsToRosMsg(obss[i]));
        }
        //ros publish
        pub_obss.publish(obss_msg);
        std::cout<< "pub obss" << std::endl;
        sleep(1);
        //for next batch
        obss.clear();
        obss_msg.MultiObs.clear();
     }

     //push
     obss.push_back(obs_single);
     /*
     if(!if_start)
       if_start= true;
     */
   }//while ends
 }

 void ObsFromFile::WpCurrCb(const uascode::WpCurrent::ConstPtr &msg)
 {
    seq_current= msg->wp_current;

    std::cout<<"current waypoint #: "
             << seq_current
             << std::endl;

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
