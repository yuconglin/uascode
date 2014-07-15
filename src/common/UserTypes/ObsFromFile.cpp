#include "ObsFromFile.hpp"
#include "fstream"
#include <vector>
#include "Planner/UserStructs/obstacle3D.h"
//ros
#include "uascode/MultiObsMsg.h"

namespace UasCode{

 //free function
 uascode::ObsMsg ObsToRosMsg(const UserStructs::obstacle3D& obs);
 //free function ends

 ObsFromFile::ObsFromFile(){
  pub_obss=nh.advertise<uascode::MultiObsMsg>("multi_obstacles",1);
 }

 ObsFromFile::~ObsFromFile(){}

 void ObsFromFile::ReadSendObss(const char *filename,int obs_num)
 {
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

     obs_single.head_xy= obs_single.head_xy/M_PI*180.;

     for(int i=0;i!= obss.size();++i)
     {
         if(obs_single.address== obss[i].address)
             repeat= true;
         break;
     }

     if(repeat){
        for(int i=0;i!= obs_num;++i){
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

   }//while ends
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
