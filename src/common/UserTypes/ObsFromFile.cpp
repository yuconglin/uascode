#include "ObsFromFile.hpp"
#include "fstream"
#include <vector>
#include "Planner/UserStructs/obstacle3D.h"
#include "common/Utils/GetTimeNow.h"
#include "common/Utils/YcLogger.h"
#include "common/UserStructs/constants.h"
#include "common/Utils/GeoUtils.h"
#include "common/Utils/FindPath.h"
//ros
#include "uascode/MultiObsMsg.h"

namespace {
    Utils::LoggerPtr s_logger(Utils::getLogger("uascode.ObsFromFile.YcLogger"));
}

namespace UasCode{

 //free function
 uascode::ObsMsg ObsToRosMsg(const UserStructs::obstacle3D& obs);
 //free function ends

 ObsFromFile::ObsFromFile():seq_current(-1)
 {
   pub_obss=nh.advertise<uascode::MultiObsMsg>("multi_obstacles",1);
   sub_wp_curr = nh.subscribe("waypoint_current",100,&ObsFromFile::WpCurrCb,this);

   for(int i=0;i!=3;++i)
       this->offsets.push_back(OffSet());
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

 void ObsFromFile::LoadOffsets(const char *filename)
 {
     std::ifstream file_offsets(filename);

     if(file_offsets.is_open())
     {
       offsets.clear();

       while(file_offsets.good() )
       {
          OffSet off_set;
          file_offsets >> off_set.x_off
                  >> off_set.y_off
                  >> off_set.z_off
                  >> off_set.hd_off;
          offsets.push_back(off_set);
       }

     }
     else{
        try {
             throw std::runtime_error ("unable to load offsets file");
         }
         catch (std::runtime_error &e) {
             std::cout << "Caught a runtime_error exception: " << e.what () << '\n';
         }
     }
 }

 void ObsFromFile::LoadOffsetsSingle(const char *filename,int idx)
 {
     std::ifstream file_offset(filename);
     std::vector<OffSet> offs;

     if(file_offset.is_open())
     {
         while(file_offset.good() )
         {
            OffSet off_set;
            file_offset >> off_set.x_off
                    >> off_set.y_off
                    >> off_set.z_off
                    >> off_set.hd_off;
            offs.push_back(off_set);
         }
         this->offsets[idx] = offs[idx];
     }
     else{
         try{
             throw std::runtime_error("unable to load offsets file:"+ (std::string)filename);
         }
         catch (std::runtime_error &e) {
             std::cout << "Caught a runtime_error exception: " << e.what() << '\n';
         }
     }

 }

 void ObsFromFile::LoadOffsets2(const char* off1, const char* off2, const char* off3)
 {
     std::string filepath = Utils::FindPath();
     // /records/offsets1200.txt
     std::string file1 = filepath + "/records/offsets" + off1 +".txt";
     std::string file2 = filepath + "/records/offsets" + off2 +".txt";
     std::string file3 = filepath + "/records/offsets" + off3 +".txt";

     UASLOG(s_logger,LL_DEBUG,file1 << "\n"
            << file2 << "\n"
            << file3 << "\n");

     this->LoadOffsetsSingle(file1.c_str(),0);
     this->LoadOffsetsSingle(file2.c_str(),1);
     this->LoadOffsetsSingle(file3.c_str(),2);
 }

 void ObsFromFile::ReadObss(const char *filename)
 {
     std::vector<UserStructs::obstacle3D> obss;
     UserStructs::obstacle3D obs_single;

     UASLOG(s_logger,LL_INFO,"ReadObss starts");

     try
     {
       obss_file.open(filename);
     }
       catch (std::ifstream::failure& e) {
           UASLOG(s_logger,LL_WARN,"Exception opening/reading file "
                  << e.what());
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

         obs_single.r = 300;
         //obs_single.hr = 70.;
         //obs_single.head_xy= Utils::_wrap_pi(M_PI/2- obs_single.head_xy * UasCode::DEG2RAD);

         UASLOG(s_logger,LL_DEBUG, "obs_single.head_xy: "<< obs_single.head_xy*M_PI/180.);

         for(int i=0;i!= obss.size();++i)
         {
             if(obs_single.address== obss[i].address)
                 repeat= true;
             break;
         }

         if(repeat){
            for(int i=0;i!= obss.size();++i)
            {
               obss[i].x1 += offsets[i].x_off;
               obss[i].x2 += offsets[i].y_off;
               obss[i].x3 += offsets[i].z_off;
               obss[i].head_xy += offsets[i].hd_off;
            }
            all_obss.push_back(obss);
            obss.clear();
         }

         //push
         obss.push_back(obs_single);
     }//while ends
     UASLOG(s_logger,LL_INFO,"ReadObss endss");
     UASLOG(s_logger,LL_DEBUG,"all_obss size: " << all_obss.size() );

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
          UASLOG(s_logger,LL_DEBUG,"send obstacles.");

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

 void ObsFromFile::SendObss2(bool f1, bool f2, bool f3)
 {
     uascode::MultiObsMsg obss_msg;

     ros::Rate r(10);
     int count=0;

     while(ros::ok())
     {
        ros::spinOnce();
        if(seq_current > 0 && count!= all_obss.size())
        {
           std::vector<UserStructs::obstacle3D> obss = all_obss[count];
           obss_msg.MultiObs.clear();

           if(f1){
             obss[0].t= Utils::GetTimeNow();
             obss_msg.MultiObs.push_back(ObsToRosMsg(obss[0]));
           }

           if(f2){
               obss[1].t= Utils::GetTimeNow();
               obss_msg.MultiObs.push_back(ObsToRosMsg(obss[1]));
           }

           if(f3){
               obss[2].t= Utils::GetTimeNow();
               obss_msg.MultiObs.push_back(ObsToRosMsg(obss[2]));
           }
           //ros publish
           pub_obss.publish(obss_msg);
           ++ count;
           sleep(1);
        }//if seq ends

     }//while ros ends

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
     UASLOG(s_logger,LL_DEBUG,"obs_msg.t: "
            << std::setprecision(4)<< std::fixed
            << obs_msg.t
            << "obs.t: " << obs.t);
     obs_msg.r= obs.r;
     obs_msg.hr= obs.hr;
     return obs_msg;
 }

}//namespace ends
