#pragma once
#include "stdint.h"
#include "UserStructs/StatusMsg.h"

namespace Utils{
  int StatusDecode(unsigned char *buf, int len, UserStructs::StatusMsg &msg);

};
