#include "ObsHelper.hpp"

namespace UasCode {

  ObsHelper::ObsHelper(const UserStructs::obstacle3D& obs3d, const double _dt):dt(_dt)
  {
     address = obs3d.address;
     reach_set = UasCode::ReachabilitySet(obs3d);
  }

  bool ObsHelper::if_print = true;

  bool ObsHelper::InSet(const UserStructs::PlaneStateSim &state)
  {
     bool if_in = GetSetPointsVh(state.t).InSetPt(state.x,state.y,state.z);

     if(if_in && if_print){
       GetSetPointsVh(state.t).PrintSet("reach_set.txt");
       if_print = false;
     }

     return if_in;
  }

  bool ObsHelper::InSet3D(double t, double x, double y, double z)
  {
     return GetSetPointsVh(t).InSetPt(x,y,z);
  }

  bool ObsHelper::NoSet(const UserStructs::PlaneStateSim &state)
  {
     return reach_set.NoSetColli( state );
  }

  void ObsHelper::CreateSetPoints()
  {
     sets.clear();
     for( double t = reach_set.Get_t0(); t!= 40; t+= dt)
     {
        reach_set.GetSet(25,t);
        sets.push_back( reach_set.AccessSet() );
     }
  }

  UserStructs::SetPointsVh ObsHelper::GetSetPointsVh(double t)
  {
     int idx =  ( t-reach_set.Get_t0() )/dt;

     if(idx < sets.size() ){
        return sets[idx];
     }
     else{
        reach_set.GetSet(25,t);
        return reach_set.AccessSet();
     }

  }

}
