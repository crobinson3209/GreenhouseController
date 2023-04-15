/*****************************************
 *
*  ______________Hardware Info______________
 * 
 *  LCD x1: I2C LCD2004, 20 cols, 4 rows,www.sunfounder.com
 *  DHT22 x2: sample at 0.5Hz (1 per 2sec)
 *  RTC x1: DS3231
 *  
 *  
 *_____________________________________________________________________________
 *
 *
*  ______________The Circuit_____________

 * 
 * DHT22s:
 *   Connect a 10K pullup resistor DHT22 data to  DHT22 5V power for each DHT sensor
 *   DHT22_0 VCC pin to 5V
 *   DHT22_0 GND pin to ground
 *   DHT22_0 DATA pin to pin 12 (+pullup 10kΩ to 5V)
 *   DHT22_1 VCC pin to 5V
 *   DHT22_1 GND pin to ground
 *   DHT22_1 DATA pin to pin 13 (+pullup 10kΩ to 5V) 
 * 
 * RTC (DS3231):
 *   VCC line to 5V
 *   GND line to GND
 *   SDA line to SDA / pin 20 on MEGA (or A4 on other) +10kΩ pullup to 5V (same resistor as for LCD)
 *   SCL line to SCL / pin 21 on MEGA (or A5 on other) +10kΩ pullup to 5V (same resistor as for LCD)
 *   
 * LEDs (WS2811)
 *   VCC line to 12v Power Supply
 *   GND line to 12vPS GND (then to common ground)
 *   Data line to 220Ω resistor then to Pin 
 
*****************************************/


//_____________________________________________________________________________


// ___Libraries___
  // on the use of "xxx.h" versus <xxx.h>:
      // < > : forces compiler to search default #include directory
      // " " : tells compiler to search current working directory, then default directory if first search fails
  #include "Wire.h"
  #include "SPI.h"
  #include "Adafruit_Sensor.h"
  #include "RTClib.h"
  #include "DHT.h"
  #include "DHT_U.h"
  #include "Adafruit_BME280.h"
  #include "DataPoint.h"
//******************************


// ____DHTs____
  // set up DHT1
  #define DHT1_PIN 12     // Digital pin connected to the DHT sensor    MEGA:12 / UNO:
  #define DHT1_TYPE DHT22   // DHT 22  (AM2302), AM2321    
  DHT_Unified dht1(DHT1_PIN, DHT1_TYPE); // Initialize DHT sensor 1

  // set up DHT2
  #define DHT2_PIN 13     // Digital pin connected to the DHT sensor    MEGA:13 / UNO:
  #define DHT2_TYPE DHT22   // DHT 22  (AM2302), AM2321    
  DHT_Unified dht2(DHT2_PIN, DHT2_TYPE); // Initialize DHT sensor 2

  uint32_t dht1_delayMS; // create variable to hold delay time for DHTs
  uint32_t dht2_delayMS; // create variable to hold delay time for DHTs

  bool dht1_temp_success = false;
  bool dht1_hum_success = false;
  bool dht2_temp_success = false;
  bool dht2_hum_success = false;
//******************************


/*
//___Fans___
  #define FAN1_ENABLE 11
  #define FAN1_DIRA 9
  #define FAN1_DIRB 10
//******************************
*/


// ___RELAYs___
  #define heater1_PIN 3
  #define circulatingFAN_PIN 4
  #define ventFAN_PIN 5
  #define light1_PIN 6
  
//******************************


// ___Global Variables___
  float minTempF, setTempF, maxTempF, minTempC, maxTempC, setTempC;
  float minHum, setHum, maxHum;
  float g_dht1_hum, g_dht2_hum, g_dht1_tempc, g_dht2_tempc;
  bool isCFan_ON, isVFan_ON, isHeater1_ON, isLight1_ON;
//******************************



//_____________________________________________________________________________



void setup() 
{
  // Startup delay and Serial begin  
    delay(3000); // power-up safety delay
    Serial.begin(9600);
  //******************************




  //___DHTs___
    Serial.println(F("Initializing DHT sensors 1 & 2..."));

    // initialize dht sensors and create sensor objects from unified Adafruit_Sensor library
      dht1.begin(); //initiallize dht1
      sensor_t sensor_dht1; // create sensor object for dht1
      dht2.begin(); //initialize dht2
      sensor_t sensor_dht2; // create sensor object for dht2

    // get sensor data
      dht1.temperature().getSensor(&sensor_dht1);
      dht1.humidity().getSensor(&sensor_dht1);
      dht2.temperature().getSensor(&sensor_dht2);
      dht2.humidity().getSensor(&sensor_dht2);

      dht1_delayMS = sensor_dht1.min_delay / 1000; // set delay between sensor readings based on sensor details
      dht2_delayMS = sensor_dht2.min_delay / 1000; // set delay between sensor readings based on sensor details

    // print the sensor data retrieved above
        Serial.println(F("------------------------------------"));
        Serial.println(F("DHT Sensor #1:"));
        Serial.println(F("Temperature Sensor"));
        Serial.print  (F("Sensor Type: ")); Serial.println(sensor_dht1.name);
        Serial.print  (F("Driver Ver:  ")); Serial.println(sensor_dht1.version);
        Serial.print  (F("Unique ID:   ")); Serial.println(sensor_dht1.sensor_id);
        Serial.print  (F("Max Value:   ")); Serial.print(sensor_dht1.max_value); Serial.println(F("°C"));
        Serial.print  (F("Min Value:   ")); Serial.print(sensor_dht1.min_value); Serial.println(F("°C"));
        Serial.print  (F("Resolution:  ")); Serial.print(sensor_dht1.resolution); Serial.println(F("°C"));
        Serial.println(F("Humidity Sensor"));
        Serial.print  (F("Sensor Type: ")); Serial.println(sensor_dht1.name);
        Serial.print  (F("Driver Ver:  ")); Serial.println(sensor_dht1.version);
        Serial.print  (F("Unique ID:   ")); Serial.println(sensor_dht1.sensor_id);
        Serial.print  (F("Max Value:   ")); Serial.print(sensor_dht1.max_value); Serial.println(F("%"));
        Serial.print  (F("Min Value:   ")); Serial.print(sensor_dht1.min_value); Serial.println(F("%"));
        Serial.print  (F("Resolution:  ")); Serial.print(sensor_dht1.resolution); Serial.println(F("%"));
        Serial.println(F("------------------------------------"));
        Serial.println(F(" "));
        Serial.println(F("------------------------------------"));
        Serial.println(F("DHT Sensor #2:"));
        Serial.println(F("Temperature Sensor"));
        Serial.print  (F("Sensor Type: ")); Serial.println(sensor_dht2.name);
        Serial.print  (F("Driver Ver:  ")); Serial.println(sensor_dht2.version);
        Serial.print  (F("Unique ID:   ")); Serial.println(sensor_dht2.sensor_id);
        Serial.print  (F("Max Value:   ")); Serial.print(sensor_dht2.max_value); Serial.println(F("°C"));
        Serial.print  (F("Min Value:   ")); Serial.print(sensor_dht2.min_value); Serial.println(F("°C"));
        Serial.print  (F("Resolution:  ")); Serial.print(sensor_dht2.resolution); Serial.println(F("°C"));
        Serial.println(F("Humidity Sensor"));
        Serial.print  (F("Sensor Type: ")); Serial.println(sensor_dht2.name);
        Serial.print  (F("Driver Ver:  ")); Serial.println(sensor_dht2.version);
        Serial.print  (F("Unique ID:   ")); Serial.println(sensor_dht2.sensor_id);
        Serial.print  (F("Max Value:   ")); Serial.print(sensor_dht2.max_value); Serial.println(F("%"));
        Serial.print  (F("Min Value:   ")); Serial.print(sensor_dht2.min_value); Serial.println(F("%"));
        Serial.print  (F("Resolution:  ")); Serial.print(sensor_dht2.resolution); Serial.println(F("%"));
        Serial.println(F("------------------------------------"));

        
  //******************************

  /*
  //___FANs___
    Serial.println(F("Initializing Air Control..."));
    pinMode(8, OUTPUT); 
    pinMode(FAN1_ENABLE,OUTPUT); //set pinmodes for fan
    pinMode(FAN1_DIRA,OUTPUT); //set pinmodes for fan
    pinMode(FAN1_DIRB,OUTPUT); //set pinmodes for fan

    pinMode(9, OUTPUT); 
    pinMode(FAN2_ENABLE,OUTPUT); //set pinmodes for fan
    pinMode(FAN2_DIRA,OUTPUT); //set pinmodes for fan
    pinMode(FAN2_DIRB,OUTPUT); //set pinmodes for fan
  //******************************
  */

  //___RELAYS___
    pinMode(heater1_PIN, OUTPUT);
    pinMode(circulatingFAN_PIN, OUTPUT);
    pinMode(ventFAN_PIN, OUTPUT);
    pinMode(light1_PIN, OUTPUT);
  //******************************


  // Set Environmental Parameter SetPoints (hard-coded now, TODO: make editable with menu on LCD)
    // Temperature
      minTempF = 50;
      maxTempF = 80;
      setTempF = 75;

      minTempC = (minTempF - 32) * (5/9);
      maxTempC = (maxTempF - 32) * (5/9);
      setTempC = (setTempF - 32) * (5/9);
      
    // Humidity, in percent
      minHum = 65;
      maxHum = 85;
      setHum = 75;
      
    // States
      isCFan_ON = false;
      isVFan_ON = false;
      isHeater1_ON = false;
      isLight1_ON = false; 
      
  //******************************

}


//_____________________________________________________________________________

void loop() 
{

  /*
  //___HEAT___  
  if(tempfAvg <= setTemp){
    digitalWrite(heater1_PIN, HIGH);
    isHeater1_ON = true;
  }

  if(tempf1 >= maxTemp || tempf2 >= maxTemp){
    digitalWrite(heater1_PIN, LOW);
    isHeater1_ON = false;
  }
  */
  // check if time elasped 
  readSensors();
  setStates();
  takeAction();
  //logData()

  printAll();


}


//_____________________________________________________________________________

void readSensors()
{
  readDHTs(); // read temp and humidity from DHT22 sensors
  readBME280s(); // read temp, humidity, and pressure from BME280 sensors
  readLights(); // read light level
  // readMoisture();
}


void readDHTs()
{
  //  if(updateDHTs(); // updateDHT values if time since last update > 2sec
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
  delay(dht1_delayMS);

  // get temp and humidity events
    sensors_event_t event_dht1;
    sensors_event_t event_dht2;

  // read DHTs and set booleans for successful read
    // DHT1
    dht1.temperature().getEvent(&event_dht1);
      if (isnan(event_dht1.temperature)) 
      {
         Serial.println(F("Error reading DHT1 Temperature!"));
         dht1_temp_success = false;
      }
      else 
      { 
        dht1_temp_success = true; 
        g_dht1_tempc = event_dht1.temperature;
      }
    dht1.humidity().getEvent(&event_dht1);
      if (isnan(event_dht1.relative_humidity)) 
      {
         Serial.println(F("Error reading DHT1 Humidity!"));
         dht1_hum_success = false;
      }
      else 
      { 
        dht1_hum_success = true; 
        g_dht1_hum = event_dht1.relative_humidity;
      }
      
    // DHT2
    dht2.temperature().getEvent(&event_dht2);
      if (isnan(event_dht2.temperature)) 
      {
         Serial.println(F("Error reading DHT2 Temperature!"));
         dht2_temp_success = false;
      }
      else 
      { 
        dht2_temp_success = true; 
        g_dht2_tempc = event_dht2.temperature;
      }
    dht2.humidity().getEvent(&event_dht2);
      if (isnan(event_dht2.relative_humidity)) 
      {
         Serial.println(F("Error reading DHT2 Humidity!"));
         dht2_hum_success = false;
      }
      else 
      { 
        dht2_hum_success = true; 
        g_dht2_hum = event_dht1.relative_humidity;
      }
}

void readBME280s()
{
  
}
void readLights()
{
  // nothing here yet
}



void setStates()
{
  // set fan states
  if( (g_dht1_tempc >= setTempC) && (g_dht2_tempc >= setTempC) )  // TODO: add BME temp reading
  {
    // turn on cooling fan until temp is _____?
    // make sure heat is off 
      isHeater1_ON = false;
  }
  else
  {
    // set heat states
  }
}

void takeAction()
{
  if(!isHeater1_ON) // if HEAT1 should be OFF
  { 
    digitalWrite(heater1_PIN, LOW); // turn off
    delay(1000); // allow time for pin to change
  }

}


void printAll()
{

  
/*
  Serial.print(F("TimePoint: "));
  Serial.print(currentTIME.hour(), DEC);
  Serial.print(':');
  Serial.print(currentTIME.minute(), DEC);
  Serial.print(':');
  Serial.print(currentTIME.second(), DEC);
  Serial.println();
*/
  // Print DHT Sensor Values
    // from DHT1
    if (dht1_temp_success && dht1_hum_success)
    {
      Serial.print(F("DHT1 Temperature: "));
      Serial.print(g_dht1_tempc);
      Serial.println(F("°C"));
      Serial.print(F("DHT1 Humidity: "));
      Serial.print(g_dht1_hum);
      Serial.println(F("%"));
    }
    else { Serial.println(F("Error with DHT1 sensor!")); }

    // from DHT2
    if (dht2_temp_success && dht2_hum_success)
    {
      Serial.print(F("DHT2 Temperature: "));
      Serial.print(g_dht2_tempc);
      Serial.println(F("°C"));
      Serial.print(F("DHT2 Humidity: "));
      Serial.print(g_dht2_hum);
      Serial.println(F("%"));
    }
    else { Serial.println(F("Error with DHT2 sensor!")); }
        

}
