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


//___RTC___
  RTC_DS3231 rtc;
  char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
//******************************


//____LCD____
  LiquidCrystal_I2C lcd(0x27,20,4);  // The LCD address is 0x27 (a 20 chars and 4 line display)
//******************************


// ____DHTs____
  // set up DHT0
  #define DHT0_PIN 12     // Digital pin connected to the DHT sensor    MEGA:12 / UNO:
  #define DHT0_TYPE DHT22   // DHT 22  (AM2302), AM2321    
  DHT dht0(DHT0_PIN, DHT0_TYPE); // Initialize DHT0 sensor

  // set up DHT1
  #define DHT1_PIN 13     // Digital pin connected to the DHT sensor    MEGA:13 / UNO:
  #define DHT1_TYPE DHT22   // DHT 22  (AM2302), AM2321    
  DHT dht1(DHT1_PIN, DHT1_TYPE); // Initialize DHT1 sensor
//******************************


/*
//___Fans___
  #define FAN0_ENABLE 11
  #define FAN0_DIRA 9
  #define FAN0_DIRB 10
//******************************
*/


// ___RELAYs___
  #define HEAT0_PIN 3
  #define FAN0_PIN 4
  #define FAN1_PIN 5
  #define LIGHT0_PIN 6
//  #define BSF0LIGHT_PIN #
//  #define BSF0
//  #define WATER0HEAT_PIN #
//  #define WATER0AIR_PIN #
//  #define WATER0PUMP_PIN #
//  
//******************************


// ___Global Variables___
  volatile int g_flag_timer2;
  int dawnHour, midDayHour, duskHour, photoPeriod; 
  int logInterval = 60; // seconds between logs
  float minTemp, setTemp, maxTemp, setHum, maxHum;
  float HUM_0, HUM_1, TEMP_0, TEMP_1, TEMPF_0, TEMPF_1;
  bool FAN0_STATE, FAN1_STATE, HEAT0_STATE;
  DateTime currentTIME = rtc.now(); // create object so it can be edited anywhere
  DateTime prevTIME = rtc.now(); // create object so it can be edited anywhere
  TimeSpan deltaTIME = currentTIME - prevTIME; // create object so it can be edited anywhere
//******************************



//_____________________________________________________________________________



void setup() 
{
  // initialize variables
  g_flag_timer2 = 0;

  delay(3000); // power-up safety delay
  Serial.begin(9600);

  //Timer2 (interrupt each 30s)
  // interrupt time = 1/(16Mhz/1024) * 521 =  30s;
  TCCR2A = 0;                 // Reset entire TCCR1A to 0 
  TCCR2B = 0;                 // Reset entire TCCR1B to 0
  TCCR2B |= B00000111;        //Set CS20, CS21 and CS22 to 1 so we get prescalar 1024  
  TIMSK2 |= B00000100;        //Set OCIE1B to 1 so we enable compare match B
  OCR2B = 521;                //Finally we set compare register B to this value 
  sei();                      //Enable back the interrupts

  midDayHour = dawnHour + (photoPeriod / 2);

// ___RTC___
  #ifndef ESP8266 //if there is no ESP8266 defined
    while (!Serial) // wait for serial port to connect. Needed for native USB
  #endif

  Serial.println(F("Initializing RTC Module..."));
   if (! rtc.begin()) {    // make sure RTC module is working
     Serial.println("Couldn't find RTC");
     Serial.flush();
     abort();
   }

   if (rtc.lostPower()) {
     Serial.println("RTC lost power, setting time!");
     // When time needs to be set on a new device, or after a power loss, the following line sets the RTC to the date & time this sketch was compiled
     rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
     // This line sets the RTC with an explicit date & time, for example to set January 21, 2014 at 3am you would call: rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
   }


//******************************

//___LCD___
  Serial.println(F("Initializing LCD..."));
  lcd.init();  //initialize the lcd
  lcd.backlight();  //open the backlight 
//******************************

//___DHTs___
  Serial.println(F("Initializing DHT sensors 0 & 1..."));
  dht0.begin(); //initiallize dht0
  dht1.begin(); //initialize dht1
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
  pinMode(HEAT0_PIN, OUTPUT);
  pinMode(FAN0_PIN, OUTPUT);
  pinMode(FAN1_PIN, OUTPUT);
  pinMode(LIGHT0_PIN, OUTPUT);
//******************************

}

//_____________________________________________________________________________




void loop() 
{

  if(g_timer
  // set current time and calculate time since last loop
    currentTIME = rtc.now(); // set currentTIME to...current time.
    deltaTIME = currentTIME - prevTIME; // creates a TimeSpan class object from difference between currentTIME & prevTIME
    //TimeSpan deltaTIME = currentTIME.operator-(prevTIME); // alternate way? to create a TimeSpan class object from prevTIME – currentTIME

  // if it's time for next data point, ...
    if(deltaTIME.totalseconds() >= logInterval) 
    {
      // readSensors();
      // setStates();
      // logData();

        prevTIME = currentTIME; // data has been logged, so set prevTIME as this current log point's time
    }


  //___TIME_MANAGEMENT___
    char currentTime_log[] = "DD-MM-YYYY hh-mm-ss"; //creates char object to store the time, in indicated format, for datalogging
    Serial.println(currentTIME.toString(currentTime_log)); 
    //  uint32_t loopBeginTimeUnix = LoopBeginTime.unixtime(); //create variable to store current unix time (was going to use this for ∆t but TimeSpan.operator-() fxn works better)

    int currentTime_hr = atoi((const char*) currentTIME.hour()); // converts the uint8_t buffer received from the .hour() fxn to an integer for math. or use "int num = strtol((const char*)buffer, NULL,  10)" https://arduino.stackexchange.com/questions/36398/how-to-convert-uint8-to-int-for-maths-operation-in-arduino
 

  // TODO: set temp schedule using map function
    setTemp = map(currentTime_hr, dawnHour, dawnHour+photoPeriod, minTemp, maxTemp);
  





  /*
  //___HEAT___  
  if(tempfAvg <= setTemp){
    digitalWrite(HEAT0_PIN, HIGH);
    HEAT0_STATE = true;
  }

  if(tempf0 >= maxTemp || tempf1 >= maxTemp){
    digitalWrite(HEAT0_PIN, LOW);
    HEAT0_STATE = false;
  }
  */



  /*
  //___Record End Loop Time and Log Data___
  DateTime TIME_loopEnd = rtc.now(); //creates DateTime class object based on current time at this stage (end) of loop
    char loopEndT_log[] = "DD-MM-YYYY hh-mm-ss"; //creates char object to store the time, in indicated format, for datalogging
    Serial.println(TIME_loopEnd.toString(loopEndT_log)); 
    uint32_t UTIME_loopEnd = TIME_loopEnd.unixtime(); //create variable to store current unix time

  DateTime TIME_loopBegin;
  TimeSpan deltaTIME = TIME_loopEnd.operator-(TIME_loopBegin); // creates a TimeSpan class object
  // TODO: need to make this timespan object usable. pull individual h/m/s and concatenate into char or string
  */



  //readSensors
  //logData()
  //setStates
  



}


//_____________________________________________________________________________

void readSensors()
{
  //___DHTs___  
    delay(2000);  // Wait a few seconds between measurements.

  //  if(updateDHTs(); // updateDHT values if time since last update > 2sec
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
    float hum0 = dht0.readHumidity();  // Read humidity
    float temp0 = dht0.readTemperature();  // Read temperature as Celsius (the default)
    float tempf0 = dht0.readTemperature(true); // Read temperature as Fahrenheit (isFahrenheit = true)

    float hum1 = dht1.readHumidity(); // Read humidity
    float temp1 = dht1.readTemperature();  // Read temperature as Celsius (the default) 
    float tempf1 = dht1.readTemperature(true); // Read temperature as Fahrenheit (isFahrenheit = true)

    float rtctemp = rtc.getTemperature();

  // Check if any reads failed.
    if (isnan(hum0) || isnan(temp0) || isnan(tempf0)) {
      Serial.println(F("Failed to read from DHT sensor #0!"));
      
    }
    if (isnan(hum1) || isnan(temp1) || isnan(tempf1)) {
      Serial.println(F("Failed to read from DHT sensor #1!"));
    }

    float hif0 = dht0.computeHeatIndex(tempf0, hum0);  // Compute heat index in Fahrenheit (the default)

    float hif1 = dht1.computeHeatIndex(tempf1, hum1);  // Compute heat index in Fahrenheit (the default)

    float humAvg = (hum0 + hum1) / 2;
    float tempfAvg = (tempf0 + tempf1) / 2;
//******************************

}

//_____________________________________________________________________________


// setup Timer2 Interrupt function
  ISR(TIMER2_COMPB_vect)
  {                               
   // set flag
   g_flag_timer2 = 1;
  }


void setStates()
{
  if(!HEAT0_STATE) // if HEAT0 should be OFF
  { 
    digitalWrite(HEAT0_PIN, LOW); // turn off
    delay(3000); // allow time for pin to change
  }
  else // if HEAT0 should be ON
  { 
    digitalWrite(HEAT0_PIN, HIGH); // turn ON
    delay(3000); // allow time for pin to change
  }
  
}









/*  
  lcd.setCursor(0,0); // set cursor to column 0, line 0
  lcd.print(F("H0: "));
  lcd.print(hum0);
  lcd.print(F("%"));
  lcd.setCursor(0,1); // set cursor to column 0, line 1
  lcd.print(F("T0: "));
  lcd.print(tempf0);
  lcd.print((char)223); //prints the degree (˚) symbol
  lcd.print(F("F"));
  lcd.setCursor(0,2); // set cursor to column 0, line 0
  lcd.print(F("H1: "));
  lcd.print(hum1);
  lcd.print(F("%"));
  lcd.setCursor(0,3); // set cursor to column 0, line 1
  lcd.print(F("T1: "));
  lcd.print(tempf1);
  lcd.print((char)223); //prints the degree (˚) symbol
  lcd.print(F("F"));

  Serial.print(F("TimePoint: "));
  Serial.print(currentTIME.hour(), DEC);
  Serial.print(':');
  Serial.print(currentTIME.minute(), DEC);
  Serial.print(':');
  Serial.print(currentTIME.second(), DEC);
  Serial.println();
  Serial.print(F("Humidity0: "));
  Serial.print(hum0);
  Serial.print(F("%  Temperature0: "));
  Serial.print(temp0);
  Serial.print(F("°C "));
  Serial.print(tempf0);
  Serial.print(F("°F  HeatIndex0: "));
  Serial.print(hif0);
  Serial.println(F("°F"));
  Serial.print(F("Humidity1: "));
  Serial.print(hum1);
  Serial.print(F("%  Temperature1: "));
  Serial.print(temp1);
  Serial.print(F("°C "));
  Serial.print(tempf1);
  Serial.print(F("°F  HeatIndex1: "));
  Serial.print(hif1);
  Serial.println(F("°F"));
  Serial.println(F("Heater on?  "));
  Serial.print(HEAT0_STATE);
*/
