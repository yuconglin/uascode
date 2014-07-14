#pragma once

#include <stdint.h>

#include "UserStructs/HeartMsg.h"

namespace Utils{
  int HeartDecode(unsigned char *buf, int len, UserStructs::HeartMsg &msg);
};
