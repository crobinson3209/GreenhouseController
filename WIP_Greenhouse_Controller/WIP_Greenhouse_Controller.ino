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
 * LCD:
 *   GND pin to ground
 *   VCC pin to 5V
 *   SDA pin to SDA/pin 20 on MEGA (or A4 on other) +10kΩ pullup to 5V (same resistor as for RTC)
 *   SCL pin to SCL/pin 21 on MEGA (or A5 on other) +10kΩ pullup to 5V (same resistor as for RTC)
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
  #include "Wire.h"
  #include "RTClib.h"
  #include "LiquidCrystal_I2C.h"
  #include "DHT.h"
  #include "DataPoint.h"
//******************************


//____LCD____
  LiquidCrystal_I2C lcd(0x27,20,4);  // The LCD address is 0x27 (a 20 chars and 4 line display)
//******************************


// ____DHTs____
  // set up DHT1
  #define DHT1_PIN 12     // Digital pin connected to the DHT sensor    MEGA:12 / UNO:
  #define DHT1_TYPE DHT22   // DHT 22  (AM2302), AM2321    
  DHT dht1(DHT1_PIN, DHT1_TYPE); // Initialize DHT sensor 1

  // set up DHT2
  #define DHT2_PIN 13     // Digital pin connected to the DHT sensor    MEGA:13 / UNO:
  #define DHT2_TYPE DHT22   // DHT 22  (AM2302), AM2321    
  DHT dht2(DHT2_PIN, DHT2_TYPE); // Initialize DHT sensor 2
//******************************


/*
//___Fans___
  #define FAN1_ENABLE 11
  #define FAN1_DIRA 9
  #define FAN1_DIRB 10
//******************************
*/


// ___RELAYs___
  #define HEAT1_PIN 3
  #define circulatingFAN_PIN 4
  #define ventFAN_PIN 5
  #define LIGHT1_PIN 6
  
//******************************


// ___Global Variables___
  float minTemp, setTemp, maxTemp;
  float minHum, setHum, maxHum;
  float HUM1, HUM2, TEMP1, TEMP2, TEMPF1, TEMPF2;
  bool circulatingFAN_STATE, ventFAN_STATE, HEAT1_STATE, LIGHT1_STATE;
//******************************



//_____________________________________________________________________________



void setup() 
{
  // Startup delay and Serial begin  
    delay(3000); // power-up safety delay
    Serial.begin(9600);


  //******************************

  //___LCD___
    Serial.println(F("Initializing LCD..."));
    lcd.init();  //initialize the lcd
    lcd.backlight();  //open the backlight 
  
  //******************************

  //___DHTs___
    Serial.println(F("Initializing DHT sensors 1 & 2..."));
    dht1.begin(); //initiallize dht1
    dht2.begin(); //initialize dht2
  
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
    pinMode(HEAT1_PIN, OUTPUT);
    pinMode(circulatingFAN_PIN, OUTPUT);
    pinMode(ventFAN_PIN, OUTPUT);
    pinMode(LIGHT1_PIN, OUTPUT);
  //******************************


  // Set Environmental Parameter SetPoints (hard-coded now, TODO: make editable with menu on LCD)
    
}

//_____________________________________________________________________________




void loop() 
{

  /*
  //___HEAT___  
  if(tempfAvg <= setTemp){
    digitalWrite(HEAT1_PIN, HIGH);
    HEAT1_STATE = true;
  }

  if(tempf1 >= maxTemp || tempf2 >= maxTemp){
    digitalWrite(HEAT1_PIN, LOW);
    HEAT1_STATE = false;
  }
  */

  readSensors();
  setStates();
  takeAction();
  //logData()

  printAll();


}


//_____________________________________________________________________________

void readSensors()
{
  readDHTs(); // read temp and humidity
  readLights(); // read light level
}


void readDHTs()
{
    //___DHTs___  
    delay(2000);  // Wait a few seconds between measurements.

  //  if(updateDHTs(); // updateDHT values if time since last update > 2sec
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
    float hum1 = dht1.readHumidity();  // Read humidity
    float temp1 = dht1.readTemperature();  // Read temperature as Celsius (the default)
    float tempf1 = dht1.readTemperature(true); // Read temperature as Fahrenheit (isFahrenheit = true)

    float hum2 = dht2.readHumidity(); // Read humidity
    float temp2 = dht2.readTemperature();  // Read temperature as Celsius (the default) 
    float tempf2 = dht2.readTemperature(true); // Read temperature as Fahrenheit (isFahrenheit = true)


  // Check if any reads failed. if any values are not a number, alert user
    if (isnan(hum1) || isnan(temp1) || isnan(tempf1)) 
      { Serial.println(F("Failed to read from DHT sensor #1!")); }
    if (isnan(hum2) || isnan(temp2) || isnan(tempf2))
      { Serial.println(F("Failed to read from DHT sensor #2!")); }

    float hif1 = dht1.computeHeatIndex(tempf1, hum1);  // Compute heat index in Fahrenheit (the default)
    float hif2 = dht2.computeHeatIndex(tempf2, hum2);  // Compute heat index in Fahrenheit (the default)

    float humAvg = (hum1 + hum2) / 2;
    float tempfAvg = (tempf1 + tempf2) / 2;
}

void readLights()
{
  // nothing here yet
}



void setStates()
{
  if(
  
}

void takeAction()
{
  if(!HEAT1_STATE) // if HEAT1 should be OFF
  { 
    digitalWrite(HEAT1_PIN, LOW); // turn off
    delay(1000); // allow time for pin to change
  }
  else // if HEAT2 should be ON
  { 
    digitalWrite(HEAT2_PIN, HIGH); // turn ON
    delay(1000); // allow time for pin to change
  }
}


void printAll()
{
  lcd.setCursor(0,0); // set cursor to column 0, line 0
  lcd.print(F("H1: "));
  lcd.print(hum1);
  lcd.print(F("%"));
  lcd.setCursor(0,1); // set cursor to column 0, line 1
  lcd.print(F("T1: "));
  lcd.print(tempf1);
  lcd.print((char)223); //prints the degree (˚) symbol
  lcd.print(F("F"));
  lcd.setCursor(0,2); // set cursor to column 0, line 0
  lcd.print(F("H2: "));
  lcd.print(hum2);
  lcd.print(F("%"));
  lcd.setCursor(0,3); // set cursor to column 0, line 1
  lcd.print(F("T2: "));
  lcd.print(tempf2);
  lcd.print((char)223); //prints the degree (˚) symbol
  lcd.print(F("F"));
  
/*
  Serial.print(F("TimePoint: "));
  Serial.print(currentTIME.hour(), DEC);
  Serial.print(':');
  Serial.print(currentTIME.minute(), DEC);
  Serial.print(':');
  Serial.print(currentTIME.second(), DEC);
  Serial.println();
*/
  Serial.println(F("Humidity1: "));
  Serial.print(hum1);
  Serial.print(F("%  Temperature1: "));
  Serial.print(temp1);
  Serial.print(F("°C "));
  Serial.print(tempf1);
  Serial.print(F("°F  HeatIndex1: "));
  Serial.print(hif1);
  Serial.println(F("°F"));
  Serial.print(F("Humidity2: "));
  Serial.print(hum2);
  Serial.print(F("%  Temperature2: "));
  Serial.print(temp2);
  Serial.print(F("°C "));
  Serial.print(tempf2);
  Serial.print(F("°F  HeatIndex2: "));
  Serial.print(hif2);
  Serial.println(F("°F"));
  Serial.println(F("Heater on?  "));
  Serial.print(HEAT1_STATE);
}
