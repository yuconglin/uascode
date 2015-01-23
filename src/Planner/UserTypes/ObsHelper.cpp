#include "ObsHelper.hpp"

namespace UasCode {

  ObsHelper::ObsHelper(const UserStructs::obstacle3D& obs3d, const double _dt):dt(_dt)
  {
     address = obs3d.address;
     reach_set = UasCode::ReachabilitySet(obs3d);
  }

  bool ObsHelper::InSet(const UserStructs::PlaneStateSim &state)
  {
     return GetSetPointsVh(state.t).InSetPt(state.x,state.y,state.z);
  }

  void ObsHelper::CreateSetPoints()
  {
     sets.clear();
     for( double t = reach_set.t0; t!= 40; t+= dt)
     {
        reach_set.GetSet(25,t);
        sets.push_back( reach_set.AccessSet() );
     }
  }

  UserStructs::SetPointsVh ObsHelper::GetSetPointsVh(double t)
  {
     int idx =  (t-t0)/dt;

     if(idx < sets.size() ){
        return sets[idx];
     }
     else{
        reach_set.GetSet(25,t);
        return reach_set.AccessSet();
     }

  }

}
