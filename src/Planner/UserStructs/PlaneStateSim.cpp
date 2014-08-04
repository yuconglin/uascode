#include "PlaneStateSim.h"
#include "common/Utils/YcLogger.h"

namespace {
  Utils::LoggerPtr s_logger(Utils::getLogger("uascode.PlaneStateSim.YcLogger"));
}

namespace UserStructs {
  PlaneStateSim::PlaneStateSim():t(0),x(0),y(0),lat(0),lon(0),z(0),
      speed(0),yaw(0),pitch(0),ax(0),ay(0),az(0){ }

  PlaneStateSim::PlaneStateSim(double _t,double _x,double _y,double _lat,double _lon,double _z,double _speed,double _yaw,double _pitch,double _ax,double _ay,double _az):
      t(_t),x(_x),y(_y),lat(_lat),lon(_lon),z(_z),speed(_speed),
      yaw(_yaw),pitch(_pitch),ax(_ax),ay(_ay),az(_az){ }

  PlaneStateSim PlaneStateSim::SmallChange(double dt){
      PlaneStateSim st;

      UASLOG(s_logger,LL_DEBUG,"speed:"<< speed<<" "
             << "pitch: "<< pitch*180./M_PI<< " "
             << "yaw: "<< yaw*180./M_PI);

      st.t= t+dt;
      st.x= x+speed*cos(pitch)*sin(yaw)*dt;
      st.y= y+speed*cos(pitch)*cos(yaw)*dt;
      st.z= z+speed*sin(pitch)*dt;
      st.GetGCS();
      st.yaw= yaw;
      st.pitch= pitch;
      st.ax= ax;
      st.ay= ay;
      st.az= az;
      return st;
  }

  void PlaneStateSim::GetUTM(){
    Utils::ToUTM(lon,lat,x,y);
  }

  void PlaneStateSim::GetGCS(){
    Utils::FromUTM(x,y,lon,lat);
  }

  void PlaneStateSim::CheckConvert(){
      if(lat==0 && lon==0){
          GetGCS();
          return;
      }
      if(x==0 && y==0){
          GetUTM();
          return;
      }
  }

}
