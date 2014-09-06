#include <GeographicLib/UTMUPS.hpp>
#include <string>
#include <cstddef>

#include "common/UserStructs/constants.h"

using namespace GeographicLib;
//zone 12N for AZ, 16N for Indiana
namespace Utils{

void ToUTM( double lon, double lat, double& x, double& y )
{
     int zone;
     bool northp;
     double gamma, k;
     UTMUPS::Forward(lat, lon, zone, northp, x, y, gamma, k);
     //x = x / 1000.;
     //y = y / 1000.;
}

void FromUTM( double x, double y, double& lon, double& lat)
{
     std::string zonestr = UasCode::ZONE;
     int zone;
     bool northp;
     UTMUPS::DecodeZone(zonestr, zone, northp);
     //x = x * 1000;
     //y = y * 1000;
     UTMUPS::Reverse(zone, northp, x, y, lat, lon);
}

}//namespace ends
