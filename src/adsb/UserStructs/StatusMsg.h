#pragma once
#include <stdint.h>
namespace UserStructs{

struct StatusMsg{
 uint8_t SeqNum;
 uint8_t GPSFix;
 uint8_t GPSMode;
 uint8_t GPSOK;
 uint8_t UTCOK;
 uint8_t SatUsed;
 uint8_t SBASSatUsed;
 uint8_t SatInView;
 float GPSHDOP;
 uint16_t count978;
 uint16_t count1090;
 uint16_t track978;
 uint16_t track1090;
 uint8_t AutoShutdown;
 uint8_t TempTooHigh;
 uint8_t ExPower;
 uint8_t ChargeStatus;
 uint8_t BatteryPercent;
 uint8_t GSCount;
 //uint64_t SerialNum;
 uint8_t NMEA_GGA;
 uint8_t NMEA_GLL;
 uint8_t NMEA_GSA;
 uint8_t NMEA_GSV;
 uint8_t NMEA_RMC;
 uint8_t NMEA_VTG;
 uint16_t MaxSupport;
 uint16_t MaxReport;
 uint8_t LEDLevel;
 uint8_t DataBurst;
 int8_t PitchOffset;

};//struct ends

};
