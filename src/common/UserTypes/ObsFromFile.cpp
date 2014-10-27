#include "ObsFromFile.hpp"
#include "fstream"
#include <stdint.h>
#include <map>
#include <vector>
#include "Planner/UserStructs/obstacle3D.h"
#include "common/Utils/GetTimeNow.h"
#include "common/Utils/YcLogger.h"
#include "common/UserStructs/constants.h"
#include "common/Utils/GeoUtils.h"
#include "common/Utils/FindPath.h"
#include "common/Utils/UTMtransform.h"
//ros
#include "yucong_rosmsg/MultiObsMsg2.h"

namespace {
    Utils::LoggerPtr s_logger(Utils::getLogger("uascode.ObsFromFile.YcLogger"));
}

namespace UasCode{

 ObsFromFile::ObsFromFile():seq_current(-1),if_mission(true),if_send_obstacle(false)
 {
   pub_obss=nh.advertise<yucong_rosmsg::MultiObsMsg2>("/mavros/multi_obstacles",1);
   sub_wp_curr = nh.subscribe("/mavros/mission_current",100,&ObsFromFile::WpCurrCb,this);

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

 void ObsFromFile::LoadOffsets2(const char* off1, const char* off2, const char* off3, const char* type)
 {
     std::string filepath = Utils::FindPath();
     // /records/offsets1200.txt
     std::string file1 = filepath + "/records/offsets_simu" + type + off1 +".txt";
     std::string file2 = filepath + "/records/offsets_simu" + type + off2 +".txt";
     std::string file3 = filepath + "/records/offsets_simu" + type + off3 +".txt";

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
         obs_single.head_xy= obs_single.head_xy * UasCode::DEG2RAD;
         //insert into address map
         if(addrs_map.find(obs_single.address)== addrs_map.end())
             addrs_map[obs_single.address] = addrs_map.size();

         //UASLOG(s_logger,LL_DEBUG, "obs_single.head_xy: "<< obs_single.head_xy);

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

 void ObsFromFile::SendObss2(bool f1, bool f2, bool f3)
 {
     yucong_rosmsg::MultiObsMsg2 obss_msg;

     ros::Rate r(10);
     int count=0;

     std::vector<uint32_t> vec_addrs;
     for (std::map<uint32_t,int>::iterator it=addrs_map.begin(); it!=addrs_map.end(); ++it)
         vec_addrs.push_back(it->first);

     bool if_start = true;

     while(ros::ok())
     {
        ros::spinOnce();

        if(if_mission){
            if_start= (seq_current >0);
        }

        if(if_start && count!= all_obss.size())
        {
           std::vector<UserStructs::obstacle3D> obss = all_obss[count];
           obss_msg.MultiObs.clear();

           bool if_pub = false;

           if(f1){
               if_pub = false;
               for(int j=0;j!= obss.size();++j)
               {
                   if(obss[j].address == vec_addrs[0]){
                       obss[j].t= Utils::GetTimeNow();
                       if(obss_log.is_open() )
                       {
                           obss_log << obss[j].address << " "
                                    << obss[j].x1 << " "
                                    << obss[j].x2 << " "
                                    << obss[j].x3 << " "
                                    << obss[j].head_xy  << " "
                                    << obss[j].speed << " "
                                    << obss[j].v_vert << " "
                                    << std::setprecision(4) << std::fixed
                                    << obss[j].t << " "
                                    << obss[j].r << " "
                                    << obss[j].hr << " "
                                    << "\n";
                       }
                       obss_msg.MultiObs.push_back(ObsToRosMsg2(obss[j]));
                       if_pub = true;
                       break;
                   }
               }
           }

           if(f2){
               if_pub = false;
               for(int j=0;j!= obss.size();++j)
               {
                   if(obss[j].address == vec_addrs[1]){
                       obss[j].t= Utils::GetTimeNow();
                       if(obss_log.is_open() )
                       {
                           obss_log << obss[j].address << " "
                                    << obss[j].x1 << " "
                                    << obss[j].x2 << " "
                                    << obss[j].x3 << " "
                                    << obss[j].head_xy  << " "
                                    << obss[j].speed << " "
                                    << obss[j].v_vert << " "
                                    << std::setprecision(4) << std::fixed
                                    << obss[j].t << " "
                                    << obss[j].r << " "
                                    << obss[j].hr << " "
                                    << "\n";
                       }
                       obss_msg.MultiObs.push_back(ObsToRosMsg2(obss[j]));
                       if_pub = true;
                       break;
                   }
               }
           }

           if(f3){
               if_pub = false;
               for(int j=0;j!= obss.size();++j)
               {
                   if(obss[j].address == vec_addrs[2]){
                       obss[j].t= Utils::GetTimeNow();
                       if(obss_log.is_open() )
                       {
                           obss_log << obss[j].address << " "
                                    << obss[j].x1 << " "
                                    << obss[j].x2 << " "
                                    << obss[j].x3 << " "
                                    << obss[j].head_xy  << " "
                                    << obss[j].speed << " "
                                    << obss[j].v_vert << " "
                                    << std::setprecision(4) << std::fixed
                                    << obss[j].t << " "
                                    << obss[j].r << " "
                                    << obss[j].hr << " "
                                    << "\n";
                       }
                       obss_msg.MultiObs.push_back(ObsToRosMsg2(obss[j]));
                       if_pub = true;
                       break;
                   }
               }
           }

           if(if_pub && if_send_obstacle)
               pub_obss.publish(obss_msg);

           ++ count;
           sleep(1);
        }//if seq ends

     }//while ros ends

 }

 void ObsFromFile::LoadSendRandom(const char *obs_file, const char *type)
 {
     const int arr[]={40,60,100,120,140,160,180,200,220,240,260};
     std::vector<int> vec (arr, arr + sizeof(arr) / sizeof(arr[0]) );

     srand (time(NULL));
     int nf0= this->RandSelectVec(vec);
     int nf1= this->RandSelectVec(vec);
     int nf2= this->RandSelectVec(vec);

     bool if0 = nf0 > 0;
     bool if1 = nf1 > 0;
     bool if2 = nf2 > 0;
     std::string off0="0",off1="0",off2="0";

     if(if0){
       off0 = this->int2string(nf0);
     }

     if(if1){
       off1 = this->int2string(nf1);
     }

     if(if2){
       off2 = this->int2string(nf2);
     }

     UASLOG(s_logger,LL_DEBUG,"random offsets:"<< off0 <<","<<off1<<","<<off2);

     this->LoadOffsets2(off0.c_str(),off1.c_str(),off2.c_str(),type);
     this->ReadObss(obs_file);
     this->SendObss2(if0,if1,if2);
 }

 void ObsFromFile::LoadSendRandomNum(const char *obs_file, int num, const char *type)
 {
     const int arr[]={40,60,100,120,140,160,180,200,220,240,260};
     std::vector<int> vec (arr, arr + sizeof(arr) / sizeof(arr[0]) );

     bool if0,if1,if2;
     std::string off0="0",off1="0",off2="0";

     srand(time(NULL));
     if(num==1)
     {
         int idx = rand() % 3;
         if(idx==0){
             if0= true;
             if1= false;
             if2= false;
         }
         else if(idx==1){
             if0= false;
             if1= true;
             if2= false;
         }
         else{
             if0= false;
             if1= false;
             if2= true;
         }
         //
     }
     else if(num==2){
         int idx = rand() % 3;
         if(idx==0){
           if0= false;
           if1= true;
           if2= true;
         }
         else if(idx==1){
           if0= true;
           if1= false;
           if2= true;
         }
         else{
           if0= true;
           if1= true;
           if2= false;
         }
         //
     }
     else{
         if0= true;
         if1= true;
         if2= true;
     }

     srand(time(NULL));
     if(if0){
       int nf0= this->RandSelectVec(vec);
       off0 = this->int2string(nf0);
     }

     if(if1){
       int nf1= this->RandSelectVec(vec);
       off1 = this->int2string(nf1);
     }

     if(if2){
       int nf2= this->RandSelectVec(vec);
       off2 = this->int2string(nf2);
     }

     UASLOG(s_logger,LL_DEBUG,"random offsets:"<< off0 <<","<<off1<<","<<off2);

     this->LoadOffsets2(off0.c_str(),off1.c_str(),off2.c_str(),type);
     this->ReadObss(obs_file);
     this->SendObss2(if0,if1,if2);
 }

 void ObsFromFile::LoadSendConfig(const char *cfg_file, const char *obs_file)
 {
    std::string file= Utils::FindPath()+"/records/"+std::string(cfg_file);
    std::ifstream config_file(file.c_str());

    int count=0;
    std::string off0,off1,off2;
    std::string type;
    bool if0, if1, if2;

    if(config_file.is_open()){
        std::string line;
        while(getline(config_file,line))
        {
            std::istringstream iss(line);
            if(count==0){
               iss >> off0 >> off1 >> off2 >> type;
            }

            if(count==1){
               iss >> if0 >> if1 >> if2;
            }
            ++count;
        }//while ends
    }//if ends

    this->LoadOffsets2(off0.c_str(),off1.c_str(),off2.c_str(),type.c_str());
    this->ReadObss(obs_file);
    this->SendObss2(if0,if1,if2);
 }

 int ObsFromFile::RandSelectVec(const std::vector<int> &ints)
 {
     /* initialize random seed: */
     //srand (time(NULL));
     /* to generate a random number */
     int len = ints.size();
     int num = rand() % len;
     int idx;

     if (num > len-1)
         idx=0;
     else{
         idx= ints[num];
     }
     return idx;
 }

 std::string ObsFromFile::int2string(int _num)
 {
     std::stringstream ss;
     ss << _num;
     return ss.str();
 }


 void ObsFromFile::WpCurrCb(const std_msgs::UInt16::ConstPtr &msg)
 {
    seq_current= (int)msg->data;
    /*
    std::cout<<"current waypoint #: "
             << seq_current
             << std::endl;
    */
 }

 uascode::ObsMsg ObsFromFile::ObsToRosMsg(const UserStructs::obstacle3D& obs)
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

 yucong_rosmsg::ObsMsg2 ObsFromFile::ObsToRosMsg2(const UserStructs::obstacle3D &obs)
 {
     yucong_rosmsg::ObsMsg2 obs_msg;
     obs_msg.address= obs.address;
     double lon,lat;
     Utils::FromUTM(obs.x1,obs.x2,lon,lat);
     obs_msg.lat= lat;
     obs_msg.lon= lon;
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
