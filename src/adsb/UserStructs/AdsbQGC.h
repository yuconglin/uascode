#pragma once
#include <stdint.h>
namespace UserStructs{

struct AdsbQGC{
 uint32_t address;
 float latitude;
 float longitude;
 float altitude;//feet
 float hd;

};

};//namespace ends
