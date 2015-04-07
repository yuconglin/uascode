#include "ScaleObsFromFile.hpp"
#include <stdexcept>
#include <fstream>
#include <map>
#include <vector>
#include "Planner/UserStructs/obstacle3D.h"
#include "common/Utils/GetTimeNow.h"
#include "common/Utils/YcLogger.h"
#include "common/UserStructs/constants.h"
#include "common/Utils/GeoUtils.h"
#include "common/Utils/FindPath.h"
#include "common/Utils/UTMtransform.h"
//ros msgs
#include "yucong_rosmsg/MultiObsMsg2.h"
#include "yucong_rosmsg/ObsMsg2.h"

namespace {
    Utils::LoggerPtr s_logger(Utils::getLogger("uascode.ScaleObsFromFile.YcLogger"));
}

namespace UasCode{

    ScaleObsFromFile::ScaleObsFromFile():seq_current(-1),if_send_obstacle(false),if_mission(false)
    {
        pub_obss = nh.advertise<yucong_rosmsg::MultiObsMsg2>("/mavros/multi_obstacles",10);
        sub_wp_curr = nh.subscribe("/mavros/mission_current",10,&ScaleObsFromFile::mission_currentCb,this);
        sub_state = nh.subscribe("/mavros/state",10,&ScaleObsFromFile::stateCb,this);
        for(int i = 0; i != 3; ++i){
            this->offsets.push_back( OffSet() );
        }
    }

    ScaleObsFromFile::~ScaleObsFromFile(){}

    void ScaleObsFromFile::LoadOffsetsSingle(const char *filename,int idx)
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

    void ScaleObsFromFile::LoadOffsets2(const char* off1, const char* off2, const char* off3, const char* type)
    {
        std::string filepath = Utils::FindPath();
        std::string file1 = filepath + "/recordsHIL/offsets_simu" + type + off1 + ".txt";
        std::string file2 = filepath + "/recordsHIL/offsets_simu" + type + off2 + ".txt";
        std::string file3 = filepath + "/recordsHIL/offsets_simu" + type + off3 + ".txt";

        UASLOG(s_logger,LL_DEBUG,file1 << "\n"
               << file2 << "\n"
               << file3 << "\n");

        this->LoadOffsetsSingle(file1.c_str(),0);
        this->LoadOffsetsSingle(file2.c_str(),1);
        this->LoadOffsetsSingle(file3.c_str(),2);
    }

    void ScaleObsFromFile::ReadObss(const char* filename)
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

            //obs_single.r = 300;
            //obs_single.hr = 70.;
            obs_single.head_xy= obs_single.head_xy * UasCode::DEG2RAD;
            //insert into address map
            if(addrs_map.find(obs_single.address)== addrs_map.end())
                addrs_map[obs_single.address] = addrs_map.size();

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

    void ScaleObsFromFile::SendObss2(bool f1, bool f2, bool f3)
    {
        yucong_msg::MultiObsMsg2 obss_msg;
        ros::Rate r(10);
        int count = 0;

        std::vector< uint32_t > vec_addrs;
        for (std::map<uint32_t,int>::iterator it=addrs_map.begin(); it!=addrs_map.end(); ++it)
        {
            vec_addrs.push_back(it->first);
        }

        bool if_start = true;

        while( ros::ok() )
        {
            ros::spinOnce();
            if( if_mission ){
                if_start = ( seq_current > 0 );
            }

            if( if_start && count != all_obss.size() )
            {
                std::vector<UserStructs::obstacle3D> obss = all_obss[count];
                obss_msg.MultiObs.clear();

                bool if_pub = false;

                for( int j = 0; j != obss.size(); ++j )
                {
                   if( obss[j].address == vec_addr[0] && f1
                     || obss[j].address == vec_addr[1] && f2
                     || obss[j].address == vec_addr[2] && f3
                     )
                   {
                       obss[j].t = Utils::GetTimeNow();
                       obss_msg.MultiObs.push_back( ObsToMsg2( obss[j]) );
                       if_pub = true;
                   }
                }

                if( if_pub ){
                    pub_obss.publish( obss_msg );
                }

                ++ count;
                sleep(1);

            }

        }

    }

    yucong_msg::ObsMsg2 ScaleObsFromFile::ObsToMsg2( const UserStructs::obstacle3D& obs )
    {
        uascode::ObsMsg2 obs_msg;
        obs_msg.address = obs.address;
        Utils::FromUTM( obs.x1, obs.x2, obs_msg.lon, obs_msg.lat );
        obs_msg.x3 = obs.x3;
        obs_msg.head_xy = obs.head_xy;
        obs_msg.speed = obs.speed;
        obs_msg.v_vert = obs.v_vert;
        obs_msg.t = obs.t;
        UASLOG(s_logger,LL_DEBUG,"obs_msg.t"
              << std::setprecision(4)<< std::fixed
              << obs_msg.t << " "
              << obs_msg.lat << " "
              << obs_msg.lon << " "
              << obs_msg.x3 << " "
              << obs_msg.head_xy << " "
              << obs_msg.speed << " "
              << obs_msg.v_vert);
        obs_msg.r = obs.r;
        obs_msg.hr = obs.hr;
        return obs_msg;
    }

}


