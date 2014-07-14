#include "systemtime.h"

namespace Utils {

void getSystemTime (string& time_str)
{
  time_t rawtime;
  struct tm* timeinfo;
  char buffer[80];

  time (&rawtime);
  timeinfo = localtime (&rawtime);

  //strftime (buffer, 80, "%Y-%m-%d--%H-%M-%S", timeinfo);
  strftime(buffer, 80, "%Y%m%d-%H%M%S", timeinfo);
  time_str = buffer;
}


}
