#include "GetTimeUTC.h"
#include <time.h>  
#define MST (-7)
#define EST (-5)
#define TIMEZONE 0

namespace Utils{
 int GetTimeUTC()
 {
  time_t rawtime;
  struct tm * ptm;

  time ( &rawtime );

  ptm = gmtime ( &rawtime );
  return (ptm->tm_hour+TIMEZONE)*3600
         +ptm->tm_min*60
	 +ptm->tm_sec;
 }

};
