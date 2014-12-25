#include "StateUpdateSim.hpp"
#include "common/UserStructs/constants.h"
#include "common/Utils/GeoUtils.h"
#include "common/Utils/MathUtils.h"
#include "common/Utils/YcLogger.h"
#include "common/Utils/AeroCoefficients.h"
//standard
#include <cmath>

namespace{
Utils::LoggerPtr s_logger(Utils::getLogger("uascode.StateUpdateSim.YcLogger"));
}

namespace UasCode{

  StateUpdateSim::StateUpdateSim()
    :Tmax(0.),mpitch_rate(0.),myaw_rate(0.),Muav(0.),
     max_speed(0.),min_speed(0.),max_pitch(0.),min_pitch(0.){}

  StateUpdateSim::StateUpdateSim(double _Tmax,double _prate,double _yrate,double _M,
            double _max_spd,double _min_spd,double _max_pitch,double _min_pitch ):
    Tmax(_Tmax),mpitch_rate(_prate),myaw_rate(_yrate),Muav(_M),
    max_speed(_max_spd),min_speed(_min_spd),max_pitch(_max_pitch),min_pitch(_min_pitch)
    { }

  void StateUpdateSim::SetParams(double _Tmax,double _prate,double _yrate,double _M,
    double _max_spd,double _min_spd,double _max_pitch,double _min_pitch)
  {
     Tmax= _Tmax;
     mpitch_rate= _prate;
     myaw_rate= _yrate;
     Muav= _M;
     max_speed= _max_spd;
     min_speed= _min_spd;
     max_pitch= _max_pitch;
     min_pitch= _min_pitch;
  }

  UserStructs::PlaneStateSim StateUpdateSim::update(UserStructs::PlaneStateSim &st_pre,
            double nav_roll,
			double dem_yaw,
			double dem_pitch,
			double dem_thr,
                        double dt)  
  {
      //first get UTM coordinates
      if(st_pre.x==0||st_pre.y==0)
         st_pre.GetUTM();
      /*
      double K_pitch= 1.0;

      double Dpitch= K_pitch*Utils::_wrap_pi(dem_pitch-st_pre.pitch);
      Dpitch= Utils::math::constrain(Dpitch,-mpitch_rate*dt,mpitch_rate*dt);

      double pitch1= st_pre.pitch+ Dpitch;
      pitch1= Utils::math::constrain(pitch1,min_pitch,max_pitch);

      double avg_pitch= 0.5*(pitch1+st_pre.pitch);
      //2) throttle effect
      double KaV= 1;

      double rho0 = 1.225; //kg*m^-3
      double S = 10.57 * 0.092903; //m^2
      double CD0 = Utils::CD0_Drag0Lift(avg_pitch);
      double CL = Utils::CL_Lift(avg_pitch);
      double K = Utils::K_InducedDrag();
      //double kg2pl = 2.20462;
      //double m2ft = 3.28084;
      double N2pl = 0.2248;
      double Drag= 0.5 * rho0 * S * pow( st_pre.speed, 2 ) * ( CD0 + K * CL * CL )* N2pl;

      double aV= KaV*(dem_thr*Tmax - Drag - Muav*CONSTANT_G*sin(avg_pitch))/Muav;

      double speed1= st_pre.speed+aV*dt;
      speed1= Utils::math::constrain(speed1,min_speed,max_speed);

      double avg_speed= 0.5*(st_pre.speed+speed1);
      //std::cout<<"avg_speed: "<< avg_speed<< std::endl;
      double z1= st_pre.z+ avg_speed*sin(avg_pitch)*dt;
      //angular acceleration by lateral accel
      double K_yaw= 1.0;
      double lateral= CONSTANT_G*tan(nav_roll);
      //pay attention to direction
      double omega= Utils::math::sgn(lateral)/avg_speed; 
      //now let's include dem_yaw
      double u_yaw= Utils::_wrap_pi(M_PI/2.-st_pre.yaw); 
      double ud_yaw= Utils::_wrap_pi(M_PI/2.-dem_yaw);

      double Dyaw= Utils::_wrap_pi(K_yaw*(ud_yaw-u_yaw +omega*dt) );
      Dyaw= Utils::math::constrain(Dyaw,-myaw_rate*dt,myaw_rate*dt);
      //x,y
      double x1= st_pre.x+avg_speed*cos(avg_pitch)*cos(u_yaw+ Dyaw/2)*dt;
      double y1= st_pre.y+avg_speed*cos(avg_pitch)*sin(u_yaw+ Dyaw/2)*dt;

      double yaw1= Utils::_wrap_pi(u_yaw+Dyaw);
      //get accelaration
      double ax1= (speed1*cos(pitch1)*cos(yaw1)-st_pre.speed*cos(st_pre.pitch)*cos(u_yaw) )/dt;
      double ay1= (speed1*cos(pitch1)*sin(yaw1)-st_pre.speed*cos(st_pre.pitch)*sin(u_yaw) )/dt;
      double az1= (speed1*sin(pitch1)-st_pre.speed*sin(st_pre.pitch) )/dt;
      //adjust yaw to NE coordinate
      yaw1= Utils::_wrap_pi(M_PI/2-yaw1);
      */
      double rho0 = 1.225; //kg*m^-3
      double S = 10.57 * 0.092903; //m^2
      double CD0 = Utils::CD0_Drag0Lift(0);
      double CL = Utils::CL_Lift(0);
      double K = Utils::K_InducedDrag();
      double N2pl = 0.2248;

      double D = 0.5 * rho0 * S * pow( st_pre.speed, 2 ) * ( CD0 + K * CL * CL )* N2pl;
      double L = 0.5 * rho0 * S * pow( st_pre.speed, 2 ) * CL * N2pl;

      double D_yaw = dt * L*sin(nav_roll)/(Muav*st_pre.speed*cos(st_pre.pitch) );
      std::cout<< "D_yaw:"<< D_yaw << '\n';

      D_yaw = Utils::math::constrain(D_yaw,-myaw_rate*dt,myaw_rate*dt);

      double D_pitch= dt * ( L*cos(nav_roll)-Muav*CONSTANT_G*cos(st_pre.pitch) ) / ( Muav * st_pre.speed  );

      D_pitch = Utils::math::constrain(D_pitch,-mpitch_rate*dt,mpitch_rate*dt);

      double pitch1 = st_pre.pitch + D_pitch * dt;
      pitch1 = Utils::math::constrain(pitch1,min_pitch,max_pitch);

      double yaw1 = Utils::_wrap_pi(M_PI/2-st_pre.yaw) + D_yaw * dt;

      double D_speed= dt * (dem_thr*Tmax - D - Muav*CONSTANT_G*sin(st_pre.pitch))/Muav;

      double speed1 = st_pre.speed + D_speed;

      double avg_pitch= 0.5 * st_pre.pitch + 0.5 * pitch1;
      double avg_yaw= Utils::_wrap_pi(M_PI/2-st_pre.yaw) + 0.5 * D_yaw;
      double avg_speed= st_pre.speed + 0.5 * D_speed;

      double x1 = st_pre.x + avg_speed*cos(avg_pitch)*cos(avg_yaw)*dt;
      double y1 = st_pre.y + avg_speed*cos(avg_pitch)*sin(avg_yaw)*dt;
      double z1 = st_pre.z + avg_speed*sin(avg_pitch)*dt;

      double ax1= (speed1*cos(pitch1)*cos(yaw1)-st_pre.speed*cos(st_pre.pitch)*cos(st_pre.yaw) )/dt;
      double ay1= (speed1*cos(pitch1)*sin(yaw1)-st_pre.speed*cos(st_pre.pitch)*sin(st_pre.yaw) )/dt;
      double az1= (speed1*sin(pitch1)-st_pre.speed*sin(st_pre.pitch) )/dt;
      yaw1= Utils::_wrap_pi(M_PI/2-yaw1);
      //the updated state
      UserStructs::PlaneStateSim st_new(st_pre.t+dt,x1,y1,0,0,z1,speed1,yaw1,pitch1,ax1,ay1,az1);
      st_new.GetGCS();
      return st_new;
  }

}
