#include <fstream>
#include <iostream>
#include <vector>
#include "common/Utils/FindPath.h"
#include "UserStructs/obstacle3D.h"

int main(int argc,char** argv)
{
    std::string obs_file = Utils::FindPath()+"records/"+argv[1];
    std::string obs_file_whole = Utils::FindPath()+"records/pp_"+argv[1];
    std::ifstream f_obs(obs_file.c_str());
    std::ofstream fw_obs(obs_file_whole.c_str());
    std::vector<UserStructs::obstacle3D > obs_vec;

    if(f_obs.is_open())
    {
        std::string line;
        while(getline(f_obs,line)){
            UserStructs::obstacle3D obs;
            std::istringstream iss(line);
            iss >> obs.address
                    >> obs.x1
                    >> obs.x2
                    >> obs.x3
                    >> obs.head_xy
                    >> obs.speed
                    >> obs.v_vert
                    >> obs.t >> obs.r >> obs.hr;
            obs_vec.push_back(obs);
        }
    }

    for(int i=0;i!= obs_vec.size();++i)
    {

    }

    return 0;
}
