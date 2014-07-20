#pragma once
namespace UserStructs{
struct GoalSetPt{

 double lat;
 double lon;
 double alt;

 GoalSetPt():lat(0.),lon(0.),alt(0.)
 {}

 GoalSetPt(double _lat, double _lon, double _alt):
     lat(_lat),lon(_lon),alt(_alt){ }
};

}
