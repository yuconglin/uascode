#pragma once
#include <stdint.h>
namespace UserStructs{

  struct HeartMsg{
    bool GPSValid;
    bool UTC_OK;
    uint32_t TimeStamp;
  };//struct ends

};//namespace ends
