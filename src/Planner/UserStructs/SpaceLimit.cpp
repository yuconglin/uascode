#include <stdexcept>
#include "SpaceLimit.h"
#include "common/Utils/YcLogger.h"

namespace{
 Utils::LoggerPtr s_logger(Utils::getLogger("uascode.SpaceLimit.YcLogger") );
}

namespace UserStructs{

SpaceLimit::SpaceLimit():h_upper(0.),h_lower(0.){}

SpaceLimit::~SpaceLimit(){ }

SpaceLimit::SpaceLimit(double _h_upper,double _h_lower,const std::vector<point2D> _vertex)
:h_upper(_h_upper),h_lower(_h_lower),vertex(_vertex){}//SpaceLimit ends

SpaceLimit::SpaceLimit(double _h_upper,double _h_lower):
  h_upper(_h_upper),h_lower(_h_lower){}

bool SpaceLimit::TellIn(double x, double y,double z)
{ //in return true;
  /*
  UASLOG(s_logger,LL_DEBUG,
         "h_lower: "<< h_lower
         << " h_upper: "<< h_upper
        );*/

  if( z<= h_lower ||z>= h_upper){
      UASLOG(s_logger,LL_DEBUG,"not in height: "<< z);
      return false;
  }
  if( !Utils::PointInPoly(vertex,x,y) ){
      UASLOG(s_logger,LL_DEBUG,"not in poly");
      return false;
  }
  return true;
}//TellIn ends

void SpaceLimit::LoadGeoFence(const char* filename)
{
  UASLOG(s_logger,LL_DEBUG,"load geofence");
  std::fstream myfile(filename);
  if(myfile.is_open() ){
    std::string line;
    while(std::getline(myfile,line) )
    {
        std::istringstream iss(line);
        double lat,lon,x,y;
        iss >> lat >> lon;
        Utils::ToUTM(lon,lat,x,y);
        vertex.push_back(UserStructs::point2D(x,y) );
        //std::cout<<x<<" "<<y<< std::endl;
    } //while ends

    UASLOG(s_logger,LL_DEBUG,"load geofence done");
  }//open ends

  else{
    UASLOG(s_logger,LL_DEBUG,"error:geofence file not found");
    try {
        throw std::runtime_error ("error:geofence file not found"); }
        catch (std::runtime_error &e) {
        std::cout << "Caught a runtime_error exception: " << e.what () << '\n';
    }

  }

}//LoadGeoFence ends

}
