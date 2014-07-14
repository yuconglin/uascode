#pragma once

#include <stdint.h>
//#include <math.h>
//#include <iostream>

//#include "AdsbDecode.h"
#include "UserStructs/AdsbMsg.h"

namespace Utils{
  int AdsbDecode(unsigned char *buf, int len, UserStructs::AdsbMsg &msg);
}
