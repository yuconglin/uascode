#include <map>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <iomanip>
#include <stdint.h>
#include "common/Utils/FindPath.h"
#include "UserStructs/obstacle3D.h"


//load total obstacle file and separate each obstacle into a file
int main(int argc, char** argv)
{
    std::string obs_file = Utils::FindPath()+"data/" + argv[1];
    std::ifstream f_obss(obs_file.c_str());
    std::map<uint32_t,std::vector<UserStructs::obstacle3D> > obss_map;

    while(f_obss.good())
    {
        //uint32_t addr;
        UserStructs::obstacle3D obs;
        f_obss >> obs.address
               >> obs.x1
               >> obs.x2
               >> obs.x3
               >> obs.head_xy
               >> obs.speed
               >> obs.v_vert
               >> obs.t >> obs.r >> obs.hr;

        if(obss_map.find(obs.address)== obss_map.end()){
            std::vector<UserStructs::obstacle3D> v_obss;
            v_obss.push_back(obs);
            obss_map[obs.address] = v_obss;
        }
        else{
            obss_map[obs.address].push_back(obs);
        }

    }//while ends

    //separate into different files
    for(std::map<uint32_t,std::vector<UserStructs::obstacle3D> >::iterator it= obss_map.begin();it!=obss_map.end();++it)
    {
       //transform address into string
        std::string s;
        std::stringstream out;
        out << it->first;
        s = out.str();
        std::string out_file = Utils::FindPath()+"records/obstacle_"+ s + ".txt";
        std::ofstream f_out(out_file.c_str());
        //output to a file
        std::vector<UserStructs::obstacle3D> vec_obs = it->second;
        for(int i=0;i!= vec_obs.size();++i){
            UserStructs::obstacle3D obs3d = vec_obs[i];
            f_out << obs3d.address << " "
                  << obs3d.x1 << " "
                  << obs3d.x2 << " "
                  << obs3d.x3 << " "
                  << obs3d.head_xy  << " "
                  << obs3d.speed << " "
                  << obs3d.v_vert << " "
                  << std::setprecision(4) << std::fixed
                  << obs3d.t << " "
                  << obs3d.r << " "
                  << obs3d.hr
                  << "\n";
        }
    }//for ends
    return 0;
}
