#include "fstream"
#include <iomanip>

#include "common/Utils/FindPath.h"
#include "Planner/UserStructs/obstacle3D.h"

int main(int argc, char** argv)
{
    double scale = 0.25;

    std::string filename_from = Utils::FindPath() + "records/tt_obstacle_10942331.txt";
    const char* f_from = filename_from.c_str();
    std::ifstream obss_file( f_from );

    std::string filename_to = Utils::FindPath() + "recordsHIL/scaled_obstacle_10942331.txt";
    const char* f_to = filename_to.c_str();
    std::ofstream scaled_file( f_to );

    UserStructs::obstacle3D obs_single;
    int count = 0;
    int x0, y0, z0;

    while( obss_file.good() && scaled_file.good() )
    {
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

        if( count == 0 )
        {
           x0 = obs_single.x1;
           y0 = obs_single.x2;
           z0 = obs_single.x3;
        }
        else{
           obs_single.x1 = x0 + ( obs_single.x1 - x0 ) * scale;
           obs_single.x2 = y0 + ( obs_single.x2 - y0 ) * scale;
           obs_single.x3 = z0 + ( obs_single.x3 - z0 ) * scale;
        }

        obs_single.speed *= scale;
        obs_single.v_vert *= scale;
        obs_single.r *= scale;
        obs_single.hr *= scale;

        scaled_file << obs_single.address << " "
                << std::setprecision(6) << std::fixed
                << obs_single.x1 << " "
                << obs_single.x2 << " "
                << obs_single.x3 << " "
                << obs_single.head_xy << " "
                << obs_single.speed << " "
                << obs_single.v_vert << " "
                << obs_single.t << " "
                << obs_single.r << " "
                << obs_single.hr << "\n";

        ++count;

    }
}
