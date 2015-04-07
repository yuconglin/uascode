#pragma once
#include "ros/ros.h"

#include "Planner/UserStructs/obstacle3D.h"
//ros messages
#include "yucong_rosmsg/MultiObsMsg2.h"
#include "yucong_rosmsg/ObsMsg2.h"

#include <fstream>
#include <map>

namespace UasCode{

class ScaleObsFromFile{

  public:
    ScaleObsFromFile();
    ~ScaleObsFromFile();

    //read obs
    void ReadObss(const char* filename);
    //send obs
    void SendObss2(bool f1, bool f2, bool f3);
    //load offsets from file
    void LoadOffsets2( const char* off1, const char* off2, const char* off3, const char* type = "");
    void LoadSendConfig(const char *config_file,const char *obs_file);
    void LoadSendRandom(const char *obs_file,const char* type="");
    void LoadSendRandomNum(const char *obs_file,int num,const char* type="");

    inline void SetIfMission(const bool _if_mission){this->if_mission = _if_mission;}

  private:
    struct OffSet{
        double x_off;
        double y_off;
        double z_off;
        double hd_off;

        OffSet():x_off(0),y_off(0),z_off(0),hd_off(0){}
    };
    std::map<uint32_t,int> addrs_map;
    std::vector<OffSet> offsets;

    //ros related
    ros::NodeHandle nh;
    ros::Publisher pub_obss;
    ros::Subscriber sub_wp_curr;
    ros::Subscriber sub_state;

    //variables
    std::vector<std::vector<UserStructs::obstacle3D> > all_obss;
    int seq_current;
    bool if_send_obstacle;
    bool if_mission;

    //callback functions
    void mission_currentCb(const std_msgs::UInt16::ConstPtr &msg);
    void stateCb(const mavros::State::ConstPtr &msg);

    //other functions
    yucong_msg::ObsMsg2 ObsToMsg2( const UserStructs::obstacle3D& obs );
    int RandSelectVec( const std::vector<int>& vecs );
    std::string int2string( int _num );
    void LoadOffsetsSingle(const char *filename,int idx);

    //file for obstacles
    std::ifstream obss_file;
};

}

