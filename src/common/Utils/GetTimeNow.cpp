#include <sys/time.h> 
#include <cstddef>

namespace Utils{
  double GetTimeNow()
  {
    struct timeval now;
    gettimeofday(&now, NULL);
    return (double)now.tv_sec
      +(double)now.tv_usec*1e-6;
  }

  long GetTimeLong()
  {
    struct timeval now;
    gettimeofday(&now,NULL);
    return now.tv_sec;
  }
	  
};//namespace ends
