#include <fstream>
#include <string>
#include <vector>
#include <iomanip>

#include "common/Utils/FindPath.h"
#include "UserStructs/obstacle3D.h"

//to extropolate adsb data for obstacles
int main(int argc, char** argv)
{
    std::string obs_file = Utils::FindPath()+"records/"+argv[1];
    std::string obs_file_whole = Utils::FindPath()+"records/whole_"+argv[1];
    std::ifstream f_obs(obs_file.c_str());
    std::ofstream fw_obs(obs_file_whole.c_str());
    std::vector<UserStructs::obstacle3D > obs_vec;

    //get one obstacle
    UserStructs::obstacle3D obs_pre;
    f_obs >> obs_pre.address
            >> obs_pre.x1
            >> obs_pre.x2
            >> obs_pre.x3
            >> obs_pre.head_xy
            >> obs_pre.speed
            >> obs_pre.v_vert
            >> obs_pre.t >> obs_pre.r >> obs_pre.hr;
    obs_pre.head_xy= obs_pre.head_xy*M_PI/180.;
    obs_vec.push_back(obs_pre);

    while(f_obs.good())
    {
        UserStructs::obstacle3D obs;
        f_obs >> obs.address
               >> obs.x1
               >> obs.x2
               >> obs.x3
               >> obs.head_xy
               >> obs.speed
               >> obs.v_vert
               >> obs.t >> obs.r >> obs.hr;
        obs.head_xy= obs.head_xy*M_PI/180.;
        //see if extropolate is needed
        if(obs.t - obs_pre.t > 1.0){
            int cnt = (int)(obs.t-obs_pre.t);
            for(int i=0;i!=cnt-1;++i){
                UserStructs::obstacle3D obs_sub;
                double Dt= i+1;
                obs_sub.address= obs_pre.address;
                obs_sub.x1= obs_pre.x1+ obs_pre.speed*cos(obs_pre.head_xy)*Dt;
                obs_sub.x2= obs_pre.x2+ obs_pre.speed*sin(obs_pre.head_xy)*Dt;
                obs_sub.x3= obs_pre.x3+ obs_pre.v_vert*Dt;
                obs_sub.head_xy= obs_pre.head_xy;
                obs_sub.speed= obs_pre.speed;
                obs_sub.v_vert= obs_pre.v_vert;
                obs_sub.t= obs_pre.t+Dt;
                obs_sub.r= obs_pre.r;
                obs_sub.hr= obs_pre.hr;
                obs_vec.push_back(obs_sub);
            }
        }

        obs_vec.push_back(obs);
        obs_pre = obs;
    }
    //size()-1 for the zero line at the bottom
    for(int i=0;i!=obs_vec.size()-1;++i){
        UserStructs::obstacle3D obs= obs_vec[i];
        fw_obs << obs.address << " "
               << obs.x1 << " "
               << obs.x2 << " "
               << obs.x3 << " "
               << obs.head_xy  << " "
               << obs.speed << " "
               << obs.v_vert << " "
               << std::setprecision(4) << std::fixed
               << obs.t << " "
               << obs.r << " "
               << obs.hr
               << "\n";
    }

    return 0;
}
