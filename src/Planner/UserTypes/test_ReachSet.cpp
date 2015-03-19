#include "ReachabilitySet.hpp"
#include "Planner/UserStructs/obstacle3D.h"
#include <fstream>
#include <ctime>
#include <cstdlib>

int main(int argc, char** argv)
{
    double x1 = 0;
    double x2 = 0;
    double head_xy = 0./180*M_PI;
    double speed = 67;
    double x3 = 650;
    double v_vert = 2;
    double t = 120;
    double r = 300;
    double hr= 45;

    UserStructs::obstacle3D obs(12300,x1,x2,head_xy,speed,x3,v_vert,t,r,0,hr,0);

    UasCode::ReachabilitySet set(obs);

    set.GetSet(25,t+30);
    set.OutputSet("set20.txt");
    return 0;
}
