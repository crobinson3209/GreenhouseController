#include "Arduino.h"
#include "DataPOINT.h"


DataPOINT::dataPOINT(DateTime dateTIME)
{
   _dateTIME = dateTIME;
}

void DataPOINT::setDHT0(float humidity0, float temperatureFaren0, float heatIndex0)
{
  _humidity0 = humidity0;
  _temperatureFaren0 = temperatureFaren0;
  _heatIndex0 = heatIndex0;
}

void DataPOINT::setDHT1(float humidity1, float temperatureFaren1, float heatIndex1)
{  
  _humidity1 = humidity1;
  _temperatureFaren1 = temperatureFaren1;
  _heatIndex1 = heatIndex1;
}

void DataPOINT::setDHTFlags(bool DHT0FLAG, bool DHT1FLAG)
{
  _DHT0FLAG = DHT0FLAG;
  _DHT1FLAG = DHT1FLAG;
}
