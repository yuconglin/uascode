#pragma once
#include <stdint.h>
namespace UserStructs{

struct AdsbMsg{
 uint8_t alert;//traffic alert status
 uint8_t AddressType;//address type
 uint32_t address;//participant address
 float latitude;
 float longitude;
 float altitude;//feet
 uint8_t Mi; //Miscellaneous Indicator
 uint8_t NIC; //Navigation Integrity Category
 uint8_t NACp; //Navigation Accuracy Category for Position
 float v; //Horizontal velocity,kt
 float vv; //Vertical velocity, ft per minute
 float hd; //heading or tracking, north-east coordinate,degree
 uint8_t EC; //Emitter Category
 char CallSign[8]; //Call Sign
 uint8_t p; //Emergency/Priority Code
 uint8_t x; //Traffic resource
};

};
