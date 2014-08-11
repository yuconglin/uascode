#include "MissionSimFlagPt.h"

namespace UserStructs{

//constructer
MissionSimFlagPt::MissionSimFlagPt():pt(MissionSimPt()),flag(false)
{

}

MissionSimFlagPt::MissionSimFlagPt(MissionSimPt sim_pt,bool _flag):pt(sim_pt),flag(_flag)
{

}


}
