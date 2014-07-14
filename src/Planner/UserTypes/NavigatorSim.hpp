#include <fstream>
#include "armadillo"
//users
#include "UserStructs/MissionSimPt.h"
#include "UserStructs/PlaneStateSim.h"
#include "UserTypes/L1ControlSim.hpp"
#include "UserTypes/TECsSim.hpp"
#include "UserTypes/StateUpdateSim.hpp"
#include "UserStructs/obstacle3D.h"
#include "UserStructs/SpaceLimit.h"

namespace UasCode{

class NavigatorSim{
  public:
   NavigatorSim(const char* act_name="fs_action.txt",
                          const char* state_name="fs_state2.txt");
   void UpdaterSetParams(double _Tmax,double _prate,double _yrate,double _M,double _max_spd,double _min_spd,double _max_pitch,double _min_pitch);

   void TECsReadParams(const char* filename);
   void L1SetRollLim(float _roll_lim= 10./180*M_PI);
   void SetDt(double _dt);

   inline void SetSpeedTrim(double _speed_trim){ speed_trim= _speed_trim;}
   inline void SetCheckStep(double _step){check_step= _step;}
   inline void SetInserInterv(int _N){N_inter= _N;}

   void ClearRec();
   //propagate a single step
   void PropagateStep(UserStructs::PlaneStateSim& st_start,
                      UserStructs::PlaneStateSim& st_end,
		      arma::vec::fixed<2> pt_A,
		      UserStructs::MissionSimPt& pt_target);
   //propagate to a waypoint   
   //0:reach position, 1:reach length
   int PropagateWp(UserStructs::PlaneStateSim& st_start,
                    UserStructs::PlaneStateSim& st_end,
		    arma::vec::fixed<2> pt_A,
		    UserStructs::MissionSimPt& pt_target);

   //propagate to a waypoint while checking obstacle collision
   //-1:cannot reach due to collision
   //0:reach position, 1:reach length, 
   int PropWpCheck(UserStructs::PlaneStateSim& st_start,
                  UserStructs::PlaneStateSim& st_end,
		  arma::vec::fixed<2> pt_A,
		  UserStructs::MissionSimPt& pt_target,
		  std::vector<UserStructs::obstacle3D> obstacles,
		  double &length);
   //option:0:sphere,1:rectangle
   int PropWpCheck2(UserStructs::PlaneStateSim& st_start,
                  UserStructs::PlaneStateSim& st_end,
                  arma::vec::fixed<2> pt_A,
                  UserStructs::MissionSimPt& pt_target,
                  std::vector<UserStructs::obstacle3D> obstacles,
		  UserStructs::SpaceLimit spacelimit,
                  double &length,
		  int option);
   
   double GetMaxPitch(){return updater.GetMaxPitch();};

   void CopyStatesRec(std::vector<UserStructs::PlaneStateSim>& copy_rec);
   void CopyStatePart(std::vector<UserStructs::StateNode>& copy_rec); 
   void EnableAirspd();

  private:

   double dt;
   double speed_trim;
   double check_step;
   double turn_radius;
   int N_inter;
   //user types
   L1ControlSim l1_control;
   TECsSim tecs;
   StateUpdateSim updater;
   //for logging
   std::vector<UserStructs::PlaneStateSim> states_rec;
   //std::vector<UserStructs::StateNode> states_rec;
   //for goal reachable checking in planning
   //std::vector<UserStructs::PlaneStateSim> states_part_rec;
   std::vector<UserStructs::StateNode> states_part_rec;
   //fstream for logging
   std::fstream fs_act;
   std::fstream fs_state;
};

};
