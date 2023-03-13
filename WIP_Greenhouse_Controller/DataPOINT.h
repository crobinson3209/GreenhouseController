#ifndef DataPOINT_h
#define DataPOINT_h

#include "Arduino.h"
#include "Wire.h"
#include "RTClib.h"
#include "DHT.h"

class DataPOINT
{
  public:
    DataPOINT::dataPOINT(DateTime dateTIME);
    void setDHT0(float humidity0, float temperatureFaren0, float heatIndex0);
    void setDHT1(float humidity1, float temperatureFaren1, float heatIndex1);
    void setDHTFlags(bool DHT0FLAG, bool DHT1FLAG);
    
    
  private:
    DateTime _dateTIME;
    TimeSpan _timeSPAN;
    
    float _humidity0, _temperatureFaren0, _heatIndex0, _humidity1, _temperatureFaren1, _heatIndex1;
    bool _DHT0FLAG, _DHT1FLAG;

    // States
    bool _LIGHT0STATE, _HEAT0STATE, _FAN0STATE, _FAN1STATE, _VENT0STATE, _VENT1STATE;
    bool _WATER0HEATSTATE, _WATER0AIRSTATE, _WATER0PUMPSTATE;
    
};

#endif
