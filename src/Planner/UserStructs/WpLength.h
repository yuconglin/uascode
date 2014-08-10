#pragma once
#include "Planner/UserStructs/MissionSimPt.h"

namespace UserStructs{

   struct WpLength{
       UserStructs::MissionSimPt wp;
       double length;
       int result;
       //construct
       WpLength(UserStructs::MissionSimPt _wp, double _len, int _result):
     wp(_wp),length(_len),result(_result){ }
   };//struct WpLength ends

};
