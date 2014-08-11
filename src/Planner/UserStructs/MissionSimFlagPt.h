#pragma once
#include "Planner/UserStructs/MissionSimPt.h"

namespace UserStructs{

struct MissionSimFlagPt{
    UserStructs::MissionSimPt pt;
    bool flag;//ture:added point;false:original planned point

    //constructer
    MissionSimFlagPt();

    MissionSimFlagPt(MissionSimPt sim_pt,bool _flag);

};

}
