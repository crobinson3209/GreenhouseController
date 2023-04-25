/*****************************************
 *
*  ______________Hardware Info______________
 * 
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


// <p><em><strong>Libraries</strong></em></p>
  // <p>on the use of "xxx.h" versus &lt;xxx.h&gt;:</p>
      // <p>&lt; &gt; : forces compiler to search default #include directory,
      //     whereas quotes (" ") : tells compiler to search current working
      //     directory, then default directory if first search fails</p>
  #include "Wire.h"
  #include "SPI.h"
  #include "Adafruit_Sensor.h"
  #include "RTClib.h"
  #include "DHT.h"
  #include "DHT_U.h"
  #include "Adafruit_BME280.h"
  #include "DataPoint.h"
//******************************


// <p><strong><strong>DHTs</strong></strong></p>
  // <p>set up DHT1</p>
  #define DHT1_PIN 12     // Digital pin connected to the DHT sensor    MEGA:12 / UNO:
  #define DHT1_TYPE DHT22   // DHT 22  (AM2302), AM2321    
  DHT_Unified dht1(DHT1_PIN, DHT1_TYPE); // Initialize DHT sensor 1

  // <p>set up DHT2</p>
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

// <p><em><strong>BME280</strong></em></p>
  Adafruit_BME280 bme; // use I2C interface
  Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
  Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
  Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

  bool bme_temp_success = false;
  bool bme_hum_success = false;
  bool bme_pressure_success = false;
  
//******************************

// <p><em><strong>RELAYs</strong></em></p>
  #define heater1_PIN 3
  #define circulatingFAN_PIN 4
  #define ventFAN_PIN 5
  #define light1_PIN 6
  #define mister1_PIN 7
  
//******************************


// <p><em><strong>Global Variables</strong></em></p>
  float g_minTempF, g_setTempF, g_maxTempF, g_minTempC, g_maxTempC, g_setTempC;
  float g_minHum, g_setHum, g_maxHum;
  float g_dht1_hum, g_dht2_hum, g_dht1_tempc, g_dht2_tempc;
  float g_bme_hum, g_bme_tempc, g_bme_pressure;
  float g_outsideHumAvg, g_outsideHumDiff, g_outsideTempAvg, g_outsideTempDiff;
  float g_insideHumAvg, g_insideHumDiff, g_insideTempAvg, g_insideTempDiff;
  
  bool isCFan_ON, isVFan_ON, isHeater1_ON, isLight1_ON, isMister1_ON;
  int isInTestMode = 0;
//******************************



//_____________________________________________________________________________



void setup() 
{
  // <p>Startup delay and Serial begin</p>
    delay(3000); // power-up safety delay
    Serial.begin(9600);
  //******************************


  //___DHTs___
    Serial.println(F("Initializing DHT sensors 1 & 2..."));

    // <p>initialize dht sensors and create sensor objects from unified
    //     Adafruit_Sensor library</p>
      dht1.begin(); //initiallize dht1
      sensor_t sensor_dht1; // create sensor object for dht1
      dht2.begin(); //initialize dht2
      sensor_t sensor_dht2; // create sensor object for dht2

    // <p>get sensor data</p>
      dht1.temperature().getSensor(&sensor_dht1);
      dht1.humidity().getSensor(&sensor_dht1);
      dht2.temperature().getSensor(&sensor_dht2);
      dht2.humidity().getSensor(&sensor_dht2);

      dht1_delayMS = sensor_dht1.min_delay / 1000; // set delay between sensor readings based on sensor details
      dht2_delayMS = sensor_dht2.min_delay / 1000; // set delay between sensor readings based on sensor details

    // <p>print the sensor data retrieved above</p>
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

  // <p>BME280 Setup</p>
    if ( !bme.begin() ) 
    {
      Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
      while (1) delay(10);
    }
  
    bme_temp->printSensorDetails();
    bme_pressure->printSensorDetails();
    bme_humidity->printSensorDetails();
    
  //******************************


  //___RELAYS___
    pinMode(heater1_PIN, OUTPUT);
    pinMode(circulatingFAN_PIN, OUTPUT);
    pinMode(ventFAN_PIN, OUTPUT);
    pinMode(light1_PIN, OUTPUT);
    pinMode(mister1_PIN, OUTPUT);
  //******************************


  // <p>Set Environmental Parameter SetPoints (hard-coded now, TODO: make
  //     editable via a TestMode)</p>
    // <p>Temperature</p>
      g_minTempF = 50;
      g_maxTempF = 90;
      g_setTempF = 75;

      g_minTempC = (g_minTempF - 32) * (5/9);
      g_maxTempC = (g_maxTempF - 32) * (5/9);
      g_setTempC = (g_setTempF - 32) * (5/9);
      
    // <p>Humidity, in percent</p>
      g_minHum = 65;
      g_maxHum = 85;
      g_setHum = 75;
      
    // <p>States</p>
      isCFan_ON = false;
      isVFan_ON = false;
      isHeater1_ON = false;
      isLight1_ON = false; 
      
  //******************************

}


//_____________________________________________________________________________

void loop() 
{
  if (isInTestMode) 
    { getTestInputsFromSerial(); }
  else { readSensors(); }

  getAvgAndDifferential();
  
  setStates();
  
  takeAction();
  
  //logData()

  printAll();


}


//_____________________________________________________________________________

void getTestInputsFromSerial()
{
  int menuSelect = 0;
  
  Serial.println(F("TEST MODE: Ready to accept manual inputs for sensor values..."));
  while(isInTestMode)
  {
        Serial.println(F("Which sensor value would you like to change?"));
        Serial.println(F("1 - Outside Temperature (dht #2)"));
        Serial.println(F("2 - Outside Humidity (dht #2)"));
        Serial.println(F("3 - Inside Temperature1 (dht #1)"));
        Serial.println(F("4 - Inside Humidity1 (dht #1)"));
        Serial.println(F("5 - Inside Temperature2 (bme)"));
        Serial.println(F("6 - Inside Humidity2 (bme)"));
        Serial.println(F("7 - Pressure (bme)"));
        Serial.println(F("8 - _TBD_"));
        Serial.println(F("9 - _TBD_"));
    
        while (Serial.available() == 0) {}
        
        menuSelect = Serial.parseInt();
    
        switch (menuSelect) 
        {
          case 1:
            // <p>outside temperature from dht #2</p>
            Serial.print(F("Input desired Outside Temperature (dht #2): "));
            while (Serial.available() == 0) {}
            g_dht2_tempc = Serial.parseFloat();
            menuSelect = 0;
            break;
      
          case 2:
            // <p>outside humidity from dht #2</p>
            Serial.print(F("Input desired Outside Humidity (dht #2): "));
            while (Serial.available() == 0) {}
            g_dht2_hum = Serial.parseFloat();
            menuSelect = 0;
            break;
      
          case 3:
            // <p>inside temperature from dht #1</p>
            Serial.print(F("Input desired Inside Temperature (dht #1): "));
            while (Serial.available() == 0) {}
            g_dht1_tempc = Serial.parseFloat();
            menuSelect = 0;
            break;
      
          case 4:
            // <p>inside humidity from dht #1</p>
            Serial.print(F("Input desired Inside Humidity (dht #1): "));
            while (Serial.available() == 0) {}
            g_dht1_hum = Serial.parseFloat();
            menuSelect = 0;
            break;
      
          case 5: 
            // <p>inside temperature from bme</p>
            Serial.print(F("Input desired Inside Temperature (bme): "));
            while (Serial.available() == 0) {}
            g_bme_tempc = Serial.parseFloat();
            menuSelect = 0;
            break;
      
          case 6:
            // <p>inside humidity from bme</p>
            Serial.print(F("Input desired Inside Humidity (bme): "));
            while (Serial.available() == 0) {}
            g_bme_hum = Serial.parseFloat();
            menuSelect = 0;
            break;
      
          default:
            Serial.println("Please choose a valid selection");
        
        }
    Serial.println(F("Change another sensor value? Enter 1 to continue editing sensor values, and 0 to stop testing. "));
    while (Serial.available() == 0) {}
    isInTestMode = Serial.parseInt();
  }
  
}


void readSensors()
{
  readDHTs(); // read temp and humidity from DHT22 sensors
  readBME280s(); // read temp, humidity, and pressure from BME280 sensors
  readLights(); // read light level
  // <p>readSoilMoisture();</p>
}


void readDHTs()
{
  // <p>if(updateDHTs(); // updateDHT values if time since last update &gt; 2sec
  // </p>
    // <p>Reading temperature or humidity takes about 250 milliseconds! Sensor
    //     readings may also be up to 2 seconds 'old' (it's a very slow sensor)
    // </p>
    delay(dht1_delayMS);

  // <p>get temp and humidity events</p>
    sensors_event_t event_dht1;
    sensors_event_t event_dht2;

  // <p>read DHTs and set booleans for successful read</p>
    // <p>DHT1</p>
    dht1.temperature().getEvent(&event_dht1);
      if (isnan(event_dht1.temperature)) 
      {
         Serial.println(F("Error reading DHT1 Temperature!"));
         dht1_temp_success = false;
         g_dht1_tempc = -100;
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
         g_dht1_hum = -100;
      }
      else 
      { 
        dht1_hum_success = true; 
        g_dht1_hum = event_dht1.relative_humidity;
      }
      
    // <p>DHT2</p>
    dht2.temperature().getEvent(&event_dht2);
      if (isnan(event_dht2.temperature)) 
      {
         Serial.println(F("Error reading DHT2 Temperature!"));
         dht2_temp_success = false;
         g_dht2_tempc = -100;
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
         g_dht2_hum = -100;
      }
      else 
      { 
        dht2_hum_success = true; 
        g_dht2_hum = event_dht1.relative_humidity;
      }
}

void readBME280s()
{
  sensors_event_t temp_event, pressure_event, humidity_event;
  
  bme_humidity->getEvent(&humidity_event);
    if (isnan(humidity_event.relative_humidity)) 
        {
          Serial.println(F("Error reading BME Humidity!"));
           bme_hum_success = false;
           g_bme_hum = -100;
        }
    else 
        { 
          bme_hum_success = true; 
          g_bme_hum = humidity_event.relative_humidity;
        }

  bme_temp->getEvent(&temp_event);
    if (isnan(temp_event.temperature)) 
        {
          Serial.println(F("Error reading BME Temperature!"));
           bme_temp_success = false;
           g_bme_tempc = -100;
        }
    else 
        { 
          bme_temp_success = true; 
          g_bme_tempc = temp_event.temperature;
        }
      
  bme_pressure->getEvent(&pressure_event);
    if (isnan(pressure_event.pressure)) 
        {
          Serial.println(F("Error reading BME Temperature!"));
           bme_pressure_success = false;
           g_bme_pressure = -100;
        }
    else 
        { 
          bme_pressure_success = true; 
          g_bme_pressure = pressure_event.pressure;
        }
}


void readLights()
{
  // <p>nothing here yet</p>
}


void getAvgAndDifferential()
{
  // <p>calculate avg and differential for outdoor temperature</p>
      g_outsideTempAvg = (g_dht2_tempc);
      g_outsideTempDiff = 0;
      
  // <p>calculate avg and differential for indoor humidity</p>
      g_outsideHumAvg = (g_dht2_hum);
      g_outsideHumDiff = 0;
  
  // <p>calculate avg and differential for indoor temperature</p>
      g_insideTempAvg = (g_dht1_tempc + g_bme_tempc) / 2;
      g_insideTempDiff = abs(g_dht1_tempc - g_bme_tempc);
      
  // <p>calculate avg and differential for indoor humidity</p>
      g_insideHumAvg = (g_dht1_hum + g_bme_hum) / 2;
      g_insideHumDiff = abs(g_dht1_hum - g_bme_hum);
}

void setStates()
{
  // <p>set status of heater</p>
    if( g_insideTempAvg >= (g_minTempC + 10) && (g_outsideTempAvg >= g_minTempC) )
      { isHeater1_ON = false; }
    else { isHeater1_ON = true; }

  // <p>set status of ventilation fan</p>
    if ( (g_insideTempAvg >= g_outsideTempAvg) && (g_outsideTempAvg <= g_setTempC) )
      { }

  // <p>set circulating fan status</p>
    if( (g_insideTempDiff >= 7.0) || (g_insideHumDiff >= 5.0) )
    {
      isCFan_ON = true;
    }
    else { isCFan_ON = false; }

  // <p>set ventilating fan status</p>
   
}

void takeAction()
{
  if(!isHeater1_ON) // if HEAT1 should be OFF
  { 
    digitalWrite(heater1_PIN, LOW); // turn off
    delay(1000); // allow time for pin to change
  }
  else
  { 
    digitalWrite(heater1_PIN, HIGH); // turn heater relay on
    delay(1000); // allow time for pin to change
  }


}


void printAll()
{ 
/* */
  // <p>Print DHT Sensor Values</p>
    // <p>from DHT1</p>
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

    // <p>from DHT2</p>
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
        
  // <p>Print BME280 Sensor Values</p>
    if (bme_temp_success)
    {
      Serial.print(F("Temperature = "));
      Serial.print(g_bme_tempc);
      Serial.println(" *C");
    }
    else { Serial.println(F("Error with BME sensor! (temperature)")); }
    
    if (bme_hum_success)
    {
      Serial.print(F("Humidity = "));
      Serial.print(g_bme_hum);
      Serial.println(" %");
    }
    else { Serial.println(F("Error with BME sensor! (humidity)")); }

    if (bme_pressure_success)
    {
      Serial.print(F("Pressure = "));
      Serial.print(g_bme_pressure);
      Serial.println(" hPa");
    }
    else { Serial.println(F("Error with BME sensor! (pressure)")); }

    
    Serial.println();
    delay(1000);
}
