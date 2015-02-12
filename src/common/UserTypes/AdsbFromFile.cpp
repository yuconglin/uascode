#include "AdsbFromFile.hpp"
#include "common/Utils/YcLogger.h"
#include "common/Utils/FindPath.h"
#include "common/UserStructs/constants.h"
#include "common/Utils/GetTimeNow.h"
//ros
#include "uascode/MultiObsMsg.h"
//std
#include <iostream>
#include <stdlib.h>

namespace{
  Utils::LoggerPtr s_logger(Utils::getLogger("uascode.AdsbFromFile.YcLogger"));
}

namespace UasCode{
//free function
uascode::ObsMsg ObsToRosMsg(const UserStructs::obstacle3D& obs);
//free function ends

AdsbFromFile::AdsbFromFile():seq_current(-1)
{
    pub_obss=nh.advertise<uascode::MultiObsMsg>("multi_obstacles",1);
    sub_wp_curr = nh.subscribe("waypoint_current",100,&AdsbFromFile::WpCurrCb,this);

    for(int i=0;i!=3;++i)
        this->offsets.push_back(OffSet());
}

AdsbFromFile::~AdsbFromFile(){}

void AdsbFromFile::LoadOffsetsSingle(const char *filename, int idx)
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

void AdsbFromFile::LoadOffsets2(const char *off1, const char *off2, const char *off3, const char *type)
{
    UASLOG(s_logger,LL_DEBUG,"load offsets starts");
    std::string filepath = Utils::FindPath();
    // /records/offsets1200.txt
    std::string file1 = filepath + "/records/offsets_adsb" + type + off1 +".txt";
    std::string file2 = filepath + "/records/offsets_adsb" + type + off2 +".txt";
    std::string file3 = filepath + "/records/offsets_adsb" + type + off3 +".txt";

    UASLOG(s_logger,LL_DEBUG,file1 << "\n"
           << file2 << "\n"
           << file3 << "\n");

    this->LoadOffsetsSingle(file1.c_str(),0);
    this->LoadOffsetsSingle(file2.c_str(),1);
    this->LoadOffsetsSingle(file3.c_str(),2);

    UASLOG(s_logger,LL_DEBUG,"offset 0:"<< std::setprecision(6) << std::fixed
                             << offsets[0].x_off<<" "<< offsets[0].y_off<<" "<< offsets[0].z_off<<"\n"
                             << offsets[1].x_off<<" "<< offsets[1].y_off<<" "<< offsets[1].z_off<<"\n"
                             << offsets[2].x_off<<" "<< offsets[2].y_off<<" "<< offsets[2].z_off<<"\n");

    UASLOG(s_logger,LL_DEBUG,"load offsets ends");
}

void AdsbFromFile::ReadADSB(const std::vector<std::string> &file_names)
{
    UASLOG(s_logger,LL_DEBUG,"ReadADSB starts");
    for(int i=0;i!= file_names.size();++i)
    {
        UASLOG(s_logger,LL_DEBUG,"load adsb from " << file_names[i]);
        std::string file = Utils::FindPath()+"/records/"+file_names[i];
        std::ifstream adsb_file(file.c_str());
        std::vector<UserStructs::obstacle3D> vec_obs;

        if(adsb_file.is_open()){
            std::string line;
            while(getline(adsb_file,line)){
                std::istringstream iss(line);
                UserStructs::obstacle3D obs_single;
                iss >> obs_single.address
                        >> obs_single.x1
                        >> obs_single.x2
                        >> obs_single.x3
                        >> obs_single.head_xy
                        >> obs_single.speed
                        >> obs_single.v_vert
                        >> obs_single.t
                        >> obs_single.r
                        >> obs_single.hr;

                obs_single.x1 += offsets[i].x_off;
                obs_single.x2 += offsets[i].y_off;
                obs_single.x3 += offsets[i].z_off;
                obs_single.head_xy += offsets[i].hd_off;
                obs_single.r = 300;
                //obs_single.hr = 70.;
                obs_single.head_xy= obs_single.head_xy * UasCode::DEG2RAD;
                vec_obs.push_back(obs_single);
            }
        }

        UASLOG(s_logger,LL_DEBUG,"vec_obs"<<" "<<i<<" size:"<< vec_obs.size());
        this->all_obss.push_back(vec_obs);
    }
    UASLOG(s_logger,LL_DEBUG,"ReadADSB ends");
}

void AdsbFromFile::SendObss2(bool f0, bool f1, bool f2)
{
    int count=0;
    uascode::MultiObsMsg obss_msg;

    std::vector<UserStructs::obstacle3D>* obs_vec0= &all_obss[0];
    std::vector<UserStructs::obstacle3D>* obs_vec1= &all_obss[1];
    std::vector<UserStructs::obstacle3D>* obs_vec2= &all_obss[2];

    ros::Rate r(1);
    bool if_start0= f0, if_start1= f1, if_start2= f2;

    while(ros::ok())
    {
        ros::spinOnce();
        obss_msg.MultiObs.clear();
        bool if_start = seq_current>0;

        //obstacle 0
        if(f0){
            if(count >= obs_vec0->size())
              if_start0 = false;
            if(count!= obs_vec0->size() && if_start && if_start0){
                obss_msg.MultiObs.push_back(ObsToRosMsg(obs_vec0->at(count)));
            }
        }

        //obstacle 1
        if(f1){
            if(count >= obs_vec1->size())
              if_start1 = false;
            if(count!= obs_vec1->size() && if_start && if_start1){
                obss_msg.MultiObs.push_back(ObsToRosMsg(obs_vec1->at(count)));
            }
        }

        //obstacle 2
        if(f2){
            if(count >= obs_vec2->size())
              if_start2 = false;
            if(count!= obs_vec2->size() && if_start && if_start2){
                obss_msg.MultiObs.push_back(ObsToRosMsg(obs_vec2->at(count)));
            }
        }

        if(if_start && (if_start0||if_start1||if_start2) ){
            UASLOG(s_logger,LL_DEBUG,"count:"<< count);
            for(int i=0;i!=obss_msg.MultiObs.size();++i)
                obss_msg.MultiObs[i].t = Utils::GetTimeNow();
            pub_obss.publish(obss_msg);
            ++count;
        }
        r.sleep();
    }//while ros ends
}

void AdsbFromFile::LoadSendConfig(const char *filename,const std::vector<std::string> &file_names)
{
    std::string file = Utils::FindPath()+"/records/"+std::string(filename);
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
    this->ReadADSB(file_names);
    this->SendObss2(if0,if1,if2);
}

void AdsbFromFile::LoadSendRandom(const std::vector<std::string> &file_names,const char* type)
{   
    const int arr[] = {60,80,100,120,140,170,190,210,230,250};
    std::vector<int> vec (arr, arr + sizeof(arr) / sizeof(arr[0]) );

    const int arr1[]={60,80,100,120};
    std::vector<int> vec1 (arr1, arr1 + sizeof(arr1) / sizeof(arr1[0]) );
    /*for reference, don't erease
     int nf0= this->RandSelect(7,9);
     int nf1= this->RandSelect(8,9);
     int nf2= this->RandSelect(6,13);
    */

    srand (time(NULL));
    int nf0= this->RandSelectVec(vec);
    int nf1= this->RandSelectVec(vec1);
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
    this->ReadADSB(file_names);
    this->SendObss2(if0,if1,if2);
}

void AdsbFromFile::LoadSendRandomNum(const std::vector<std::string> &file_names, int num, const char *type)
{
    //const int arr[] = {60,80,100,120,140,170,190,210,230,250};
    const int arr[] = {40,50,60,70,80,90,100,110,120,130,140,150,160,170,190,210,230,250,270,280,300};
    std::vector<int> vec (arr, arr + sizeof(arr) / sizeof(arr[0]) );

    const int arr1[]={60,80,100,120};
    std::vector<int> vec1 (arr1, arr1 + sizeof(arr1) / sizeof(arr1[0]) );

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

    if(if0){
      int nf0= this->RandSelectVec(vec);
      off0 = this->int2string(nf0);
    }

    if(if1){
      int nf1= this->RandSelectVec(vec1);
      off1 = this->int2string(nf1);
    }

    if(if2){
      int nf2= this->RandSelectVec(vec);
      off2 = this->int2string(nf2);
    }

    UASLOG(s_logger,LL_DEBUG,"random offsets:"<< off0 <<","<<off1<<","<<off2);
    this->LoadOffsets2(off0.c_str(),off1.c_str(),off2.c_str(),type);
    this->ReadADSB(file_names);
    this->SendObss2(if0,if1,if2);
}

void AdsbFromFile::WpCurrCb(const uascode::WpCurrent::ConstPtr &msg)
{
   seq_current= msg->wp_current;
   /*
   std::cout<<"current waypoint #: "
            << seq_current
            << std::endl;
   */
}

uascode::ObsMsg AdsbFromFile::ObsToRosMsg(const UserStructs::obstacle3D& obs)
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
           << obs_msg.address<< " "
           << obs_msg.t <<" "
           << obs_msg.x1 <<" "
           << obs_msg.x2 <<" "
           << obs_msg.x3);
    obs_msg.r= obs.r;
    obs_msg.hr= obs.hr;
    return obs_msg;
}

int AdsbFromFile::RandSelect(int start, int end)
{
    /* initialize random seed: */
    srand (time(NULL));
    int len = end-start+1;
    int num = rand() % (len+1) + start;
    if(num== end+1)
        num=0;
    return num;
}

int AdsbFromFile::RandSelectVec(const std::vector<int> &ints)
{
    /* initialize random seed: */
    //srand (time(NULL));
    /* to generate a random number */
    int len = ints.size();
    //int num = rand() % (len+1);
    int num = rand() % len;
    int idx;

    if (num > len-1)
        idx=0;
    else{
        idx= ints[num];
    }
    return idx;
}

std::string AdsbFromFile::int2string(int _num)
{
    std::stringstream ss;
    ss << _num;
    return ss.str();
}

}
