 /*
 ==================================================================================
 Water Butt Sensor Array With COM Port API Interface
 ==================================================================================
 VCC to arduino 5v        RED WIRE
 GND to arduino GND       BLACK WIRE
 Echo to Arduino pin 10   BLUE WIRE
 Trig to Arduino pin 9    GREEN WIRE
 TEMP SENSORS pin 11      YELLOW WIRE  Dont forget the 4K7 Pullup resistor
 LED                      WHITE WIRE
 ==================================================================================
  API Functions                                                   API Command
  ---------------------------------------------------------------------
  - Turn ON Pump                    - apiPumpON         (bool)    PO
  - Turn OFF Pump                                                 PF
  - Turn ON top float switch        - apiTopFloatON     (bool)    FO
  - Turn OFF top float switch                                     FF
  - Turn ON bottom float switch     - apiBottomFloatON  (bool)    BO
  - Turn OFF bottom float switch                                  BF
  - Top temperatue                  - apiTopTemp        (float)   TT
  - Bottom temperature              - apiBottomTemp     (float)   BT
  - Turn ON UV lights               - apiUVON           (bool)    UO
  - Turn OFF UV lights                                            UF
  - Turn ON sensor LED              - apiSensorLEDON    (bool)    LO
  - Turn OFF sensor LED                                           LF
  - Read distance                   - apiDistance       (int)     RD
  - Mimic emptying                                                ME
*/

#include <Arduino.h>
#include <SPI.h>
#include <OneWire.h>              // sensors DS1820's
#include <DallasTemperature.h>    // DS18B20 Driver
#include <DHT.h>                  // DHT Driver
#include <Metro.h>                // async driver

#define ONE_WIRE_BUS 11           // DS1820 data bus
#define echoPin 10                // Echo Pin </DHT>
#define trigPin 9                 // Trigger Pin
#define LEDPin 13                 // Onboard LED
#define floatSwitchTop 7          // Top Float Switch
#define floatSwitchBottom 6       // Bottom Float Switch
#define ButtPump 4                // Water pump :)
#define uvLEDs 5                  // UV Leds
#define SensorRedLED 12           // Sensor red Led
#define BoxRedLED A2              // Red Box Side LED
#define BoxGreenLED A3            // Green Box Side LED
#define BoxBlueLED A1             // Blue Box Side LED
#define DHTPIN A0                 // DHT11 Data pin
#define DHTTYPE DHT11             // DHT11 type
#define LightSensor A5            // Light Sensor
#define SensorOrderSwap 8         // Sensor order jumper (Default N/C pulled high)

// CONSTANTS
const int  HIGH_WATER_TEMPERATURE = 30;     // Temperature of the water for high warning
const int  LOW_WATER_LEVEL = 815;           // Water butt empty level;
const int  HIGH_WATER_LEVEL  = 70;          // Water butt full to brim
const int  NIGHT_THRESHOLD_LEVEL = 18;      // 20 or less is night
const int  LOW_PUMPING_WATER_LEVEL = 180;   // level that the pumps turn off if pumping out
const int  MAXIMUM_RANGE = 2000;            // Maximum water level range needed mm
const int  MINIMUM_RANGE = 1;               // Minimum water level range needed
const bool LED_ON = HIGH;
const bool LED_OFF = LOW;
const bool PUMP_OFF = HIGH;
const bool PUMP_ON = LOW;
const bool UV_LEDS_ON = LOW;
const bool UV_LEDS_OFF = HIGH;

float dhtTemperature = 0;               // DHT Temperature
float dhtHumidity = 0;                  // DHT Humidity
float TemperatureTop = 0;               // DS TemperatureTop
float TemperatureBottom = 0;            // DS Temperature Bottom
float currentWaterLevel = 0;            // holds the current water level
float lightLevel = 800;                 // box light sensor

long  duration;                         // Duration used to calculate distance
float previousDistance = 0;
bool  showOn_UV_Code = true;
bool  showOff_UV_Code = false;
bool  TopSensorReading = false;         // Indicates which temp sensor the reading is from Top = true / Bottom =  false
bool  toggle1 = false;                  // display text backcolor
float previousLevel = 0;                // holds the previous water level before current reading
bool  filling = false;                  // water butt filling = true
bool  highWaterTemperture = false;      // high water temperature flag
bool  topFloatON = true;                // top float switch (true for overide)
bool  bottomFloatON = true;             // bottom float switch (true for override)
bool  float2ONOFF = true;               // Float switch 2
bool  float1ONOFF = true;               // Float switch 1
bool  led = false;                      // LED bit for toggles
bool  redLed = false;                   // sensor red led
bool  uvLedsONOFF = false;              // UV Leds
bool  buttPumpONOFF = false;            // Butt Pump ON/OFF
bool  PumpingUntilLowPumpLevel = false; // Flag set if level pumping has started.
byte  TemperatureTop_ByteArray[4];      // holds the 4 byte IEEE754 convertion for zigbee frame
byte  TemperatureBottom_ByteArray[4];   // holds the 4 byte IEEE754 convertion for zigbee frame
byte  currentWaterLevel_ByteArray[4];   // holds the 4 byte IEEE754 convertion for zigbee frame
byte  dhtTemperature_ByteArray[4];      // holds the 4 byte IEEE754 convertion for zigbee frame
byte  dhtHumidity_ByteArray[4];         // holds the 4 byte IEEE754 convertion for zigbee frame
byte  lightLevel_ByteArray[4];          // holds the 4 byte IEEE754 convertion for zigbee frame

bool  apiPumpON = false;                // Turns Pump ON/OFF
bool  apiTopFloatON = true;             // Turns top float ON/OFF
bool  apiBottomFloatON = true;          // Turns bottom float ON/OFF
float apiTopTemperature = 0.00;         // Top temperature (Mimic)
float apiBottomTemperature = 0.00;      // Bottom Temp (Mimic)
bool  apiUVON = false;                  // Turns UV Leds ON/OFF
bool  apiSensorLEDON = false;           // Turns the sensor LED ON/OFF
int   apiDistance = 0;                  // Water level (Mimic)
int   ledFlashBrightness = 600;         // holds the current LED brightness taken from the light sensor

DHT dht(DHTPIN, DHTTYPE);
OneWire ds(ONE_WIRE_BUS);                   // on pin A0 (a 4.7K resistor is necessary)
DallasTemperature sensors(&ds);             // Pass our oneWire reference to Dallas Temperature.

void BoxTempAndHum();
void UpdateUVLeds();
void SendXbeeDataFrame();
void bRedLED();
void GetdayOrNight();
void GetWaterTemperatures();
int GetButtWaterLevel();
void ProcessWaterLevel();

Metro getBoxTempAndHum = Metro(1800000);    // 18000000 = 30 mins
Metro uvLedPeriod = Metro(110000);          // Update UV led every 2 minuites
Metro sendDataFrame = Metro(290000);        // send data over zigbee every 5 mins
//Metro senRedLED = Metro(3000);              // 3 seconds
Metro dayOrNight = Metro(60000);            // 1 min 1 sec
Metro getTemperatures = Metro(130000);      // 2 mins 5 sec
Metro getWaterLevel = Metro(11000);          // update water level every 10 seconds

void setup(void) {
      pinMode(trigPin, OUTPUT);
      pinMode(echoPin, INPUT);
      pinMode(LEDPin, OUTPUT);              // Use LED indicator (if required) this is the pin 13 led on arduino
      pinMode(SensorRedLED,OUTPUT);
      pinMode(BoxRedLED, OUTPUT);
      pinMode(BoxGreenLED, OUTPUT);
      pinMode(BoxBlueLED, OUTPUT);
      pinMode(uvLEDs, OUTPUT);
      pinMode(ButtPump, OUTPUT);
      pinMode(SensorOrderSwap,INPUT_PULLUP);
      pinMode(floatSwitchTop, INPUT_PULLUP);
      pinMode(floatSwitchBottom, INPUT_PULLUP);

      digitalWrite(uvLEDs, UV_LEDS_OFF);    // initalise UV relay off (HIGH)
      digitalWrite(ButtPump,PUMP_OFF);
      digitalWrite(BoxRedLED, LOW);         // initalise red led
      digitalWrite(BoxGreenLED, LOW);       // initalise green led
      digitalWrite(BoxBlueLED, LOW);        // initalise blue led
      digitalWrite(SensorRedLED, LOW);      // initalise sensor red led

      Serial1.begin(9600);
      sensors.begin();                      // Start up the ds library
      dht.begin();                          // Setup DHT box sensor
      LampTest();                           // seq LEDS   RED-GREEN-BLUE -- PUMP & UVLEDS
      GetWaterTemperatures();
      ProcessWaterLevelStartupOnly();
 }

 void loop(void)
{
    // update day or night
    if (dayOrNight.check() == 1)    // check if the dayOrNight has passed its interval .
    {
        if(GetNightOrDay())         // if day
            DimFlashGreenLED(ledFlashBrightness,200);    // day light
        else  // night
            DimFlashGreenLED(ledFlashBrightness,100);     // night light level
    }

    // Update the box temperature and humidity
    if (getBoxTempAndHum.check() == 1)  // check if the GetBoxTempAndHum has passed its interval .
    {
        GetDHT11Temperature();          // get box temperature
    }

    // Update the uv leds
    if (uvLedPeriod.check() == 1)       // check if the uvLedPeriod has passed its interval .
    {
        if(GetNightOrDay())
        {
            UV_Leds(UV_LEDS_ON);
        }
        else
        {
             UV_Leds(UV_LEDS_OFF);
        }
    }

    // Update the sensor temperatures
    if (getTemperatures.check() == 1)  // check if the uvLedPeriod has passed its interval .
    {
        GetWaterTemperatures();
    }

    // Update the box red led
    // if (senRedLED.check() == 1)  // check if the uvLedPeriod has passed its interval .
    // {
    //     //if(sensorRedLed) digitalWrite(SensorRedLED,HIGH); else digitalWrite(SensorRedLED,LOW);
    //     redLed = !redLed;
    //     digitalWrite(SensorRedLED, redLed);
    // }

    // Update the water level
    if (getWaterLevel.check() == 1)  // update the water level.
    {
         ProcessWaterLevel();         // Return OK/Error  Distance value is saved to global distance variable
    }

    // Send the data
     if (sendDataFrame.check() == 1)  // check if the sendDataFrame has passed its interval .
     {
        SendDataFrame();
     }
}

void GetWaterTemperatures()
{
    sensors.requestTemperatures();      // Send the command to get temperatures
    delay(500);

    TemperatureTop = sensors.getTempCByIndex(0);        // Water Butt 1 configuration
    TemperatureBottom = sensors.getTempCByIndex(1);     // Water butt 1 configuration

     //  TemperatureTop = sensors.getTempCByIndex(1);
     //  TemperatureBottom = sensors.getTempCByIndex(0);
}

 void bRedLED()
 {
     redLed = !redLed;
     digitalWrite(SensorRedLED, redLed);
 }

 // void getWaterLevel()
 // {
 //      ProcessWaterLevel();         // Return OK/Error  Distance value is saved to global distance variable
 // }

 // void sendDataFrame()
 // {
 //     SendXbeeDataFrame();
 // }

bool GetNightOrDay()  // return true if its day
{
  lightLevel = analogRead(LightSensor);       // get the current light level
  if(lightLevel > 1020) { lightLevel = 1020;}
  if(lightLevel < 0) { lightLevel = 0;}

  ledFlashBrightness = map(lightLevel,0,1020,2,500);  // 18 = threshold day -> night

  if(lightLevel < NIGHT_THRESHOLD_LEVEL)
    return false;       // night
  else
    return true;        // day
}

void UV_Leds(bool LEDONOFF)
{
    if(!LEDONOFF)
    { // daytime
        // if(showOn_UV_Code)
        // {
            // FlashUV_ON_Code();
            // showOn_UV_Code = false;
            // showOff_UV_Code = true;
        //}
        digitalWrite(uvLEDs, UV_LEDS_ON);   // turn on UV leds
        uvLedsONOFF = UV_LEDS_ON;
    }
    else
    {  // night time
        // if(ledFlashBrightness < 2 )
        // {
        //     FlashUV_OFF_Code();
        // }
        digitalWrite(uvLEDs, UV_LEDS_OFF);   // turn off UV leds
        uvLedsONOFF = UV_LEDS_OFF;
        // showOn_UV_Code = true;
        // showOff_UV_Code = false;
    }
}

void ProcessWaterLevelStartupOnly()
{
    currentWaterLevel = GetButtWaterLevel();                 // get the current level
}

void ProcessWaterLevel()    // this is called every 1 minuite
{
    currentWaterLevel = GetButtWaterLevel();                 // get the current level

   if(currentWaterLevel != -1)                           // if data OK  -1 = error
   {
           if(currentWaterLevel < previousLevel)                // smaller value is filling up
             filling = true;
           else
             filling = false;  // emptying or static

          if(TemperatureTop > 0.5 && TemperatureBottom > 0.5)       // if temperature is not freezing
          {
             if(currentWaterLevel < HIGH_WATER_LEVEL && filling == true)    // Pumping Status
             {
                Pump(PUMP_ON);            // turn on pump if the level is high and the temperature is not freezing
                PumpingUntilLowPumpLevel = true;
             }
             else
             {
                 if(!PumpingUntilLowPumpLevel)
                 {
                     Pump(PUMP_OFF);            // turn on pump if the level is high and the temperature is not freezing
                     //PumpingUntilLowPumpLevel = false;
                 }
             }

             if(currentWaterLevel > LOW_PUMPING_WATER_LEVEL)        // Turn off pumps
             {
                Pump(PUMP_OFF);           // turn off pump
                PumpingUntilLowPumpLevel = false;
             }
          }
          else
          {
                // turn on freezing LED indications.
                Pump(PUMP_OFF);           // turn off pump
                FreezingWaterLEDs();
                UV_Leds(UV_LEDS_ON);
          }
     previousLevel = currentWaterLevel;       // save water level in previous
  }
  else
  {  // error reading distance
      Pump(PUMP_OFF);                   // turn off pump
      currentWaterLevel = 0;
      previousLevel = 0;

      for (size_t i = 0; i < 3; i++) {
          DimFlashRedLED(ledFlashBrightness,200);
       }
  }
}

int GetButtWaterLevel()
{
   digitalWrite(trigPin, LOW);
   delayMicroseconds(2);
   digitalWrite(trigPin, HIGH);
   delayMicroseconds(10); //10
   digitalWrite(trigPin, LOW);

   duration = pulseIn(echoPin, HIGH,10000);  // 1 second timeout

   //Calculate the distance (in cm) based on the speed of sound.
   currentWaterLevel = duration / 58.2; // cm
   currentWaterLevel = currentWaterLevel * 10;  //mm

   if (currentWaterLevel >= MAXIMUM_RANGE || currentWaterLevel <= MINIMUM_RANGE)
   {
       DimFlashRedLED(ledFlashBrightness,100);
      return -1;    // Send a negative number to computer and Turn LED ON to indicate "out of range"
   }
   else
   {
       DimFlashGreenLED(ledFlashBrightness,100);
       return currentWaterLevel;
   }
 }

void Pump(bool ON_OFF)
{
  if(currentWaterLevel < LOW_WATER_LEVEL)   // if there is enough water to pump then ..
  {
    if(ON_OFF) // HIGH = OFF // && bottomFloatON)
    {
        digitalWrite(ButtPump, PUMP_OFF);   // Turn OFF pump
        buttPumpONOFF = PUMP_OFF;
        //TurnOnGreenLED(LED_OFF);
    }
    else
    {
        digitalWrite(ButtPump, PUMP_ON);   // Turn ON pump
        buttPumpONOFF = PUMP_ON;
        //TurnOnGreenLED(LED_ON);
        DimFlashGreenLED(ledFlashBrightness,300);
    }
  }
  else
  {
      // Not enough water
      digitalWrite(ButtPump, PUMP_OFF);   // HARD Turn OFF pump
      buttPumpONOFF = PUMP_OFF;
      DimFlashBlueLED(ledFlashBrightness,300);
  }
}

void SendDataFrame()
{
 // To calculate the check sum you add all bytes of the packet excluding the frame delimiter 7E and the length (the 2nd and 3rd bytes).

 // ID|Temp1|Temp2|BoxTemp|BoxHumidity|Lightlevel|WaterLevel|LED|UVLed|WaterPump|Float1|Float2#
 // $001|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|x|x|x|x|x#    ( 45 bytes )

  long checkSum = 0;

  Serial1.write((byte)0x7E);    //start byte
  Serial1.write((byte)0x00);    // high part of lenght, alway zero
  Serial1.write((byte)0x3B);    // low part of lenght, number of bytes
  Serial1.write((byte)0x10);  checkSum += 0x10;   // 0x10 send request
  Serial1.write((byte)0x01);  checkSum += 0x01;   // frame id set to zero for no reply, 01 for reply

  // ID of recipient, CO-ORDINATOR
  Serial1.write((byte)0x00); checkSum += 0x00;
  Serial1.write((byte)0x13); checkSum += 0x13;
  Serial1.write((byte)0xA2); checkSum += 0xA2;
  Serial1.write((byte)0x00); checkSum += 0x00;
  Serial1.write((byte)0x41); checkSum += 0x41;
  Serial1.write((byte)0x63); checkSum += 0x63;
  Serial1.write((byte)0x18); checkSum += 0x18;
  Serial1.write((byte)0x45); checkSum += 0x45;

  // 16 bit of recipients or 0xFFFE if unkown
  Serial1.write((byte)0xFF); checkSum += 0xFF;
  Serial1.write((byte)0xFE); checkSum += 0xFE;

  Serial1.write((byte)0x00);
  Serial1.write((byte)0x00);

  //checkSum = 0x437;  // claculation of checksum from above bytes hex

  // Data frame
  // ID
  Serial1.write((byte)0x24); checkSum += 0x24; // $
  Serial1.write((byte)0x30); checkSum += 0x30; // 0
  Serial1.write((byte)0x30); checkSum += 0x30; // 0
  Serial1.write((byte)0x31); checkSum += 0x31; // 1  -- 001 = Water Butt 1
  Serial1.write((byte)0x7C); checkSum += 0x7C; // |

  // Top Temperature
   memcpy(TemperatureTop_ByteArray, &TemperatureTop,4);
   for(int i = 0; i < 4; i++)
   {
     Serial1.write((byte)TemperatureTop_ByteArray[i]);
     checkSum += TemperatureTop_ByteArray[i];
   }
   Serial1.write((byte)0x7C); checkSum += 0x7C; // |
  //---------------------------------------------------------------------------------------

  // Bottom Temperature
   memcpy(TemperatureBottom_ByteArray, &TemperatureBottom,4);
   for(int i = 0; i < 4; i++)
   {
     Serial1.write((byte)TemperatureBottom_ByteArray[i]);
     checkSum += TemperatureBottom_ByteArray[i];
   }
    Serial1.write((byte)0x7C); checkSum += 0x7C; // |
   //---------------------------------------------------------------------------------------

   // DHT11 Control Box Temperature
   memcpy(dhtTemperature_ByteArray, &dhtTemperature,4);
   for(int i = 0; i < 4; i++)
   {
      Serial1.write((byte)dhtTemperature_ByteArray[i]);
      checkSum += dhtTemperature_ByteArray[i];
   }
    Serial1.write((byte)0x7C); checkSum += 0x7C; // |
    //---------------------------------------------------------------------------------------

  // DHT11 Control Box Humidity
   memcpy(dhtHumidity_ByteArray, &dhtHumidity,4);
   for(int i = 0; i < 4; i++)
   {
     Serial1.write((byte)dhtHumidity_ByteArray[i]);
     checkSum += dhtHumidity_ByteArray[i];
   }
    Serial1.write((byte)0x7C); checkSum += 0x7C; // |
  //-----------------------------------------------------------------------------------------

  // Light Level
   memcpy(lightLevel_ByteArray, &lightLevel,4);
   for(int i = 0; i < 4; i++)
   {
     Serial1.write((byte)lightLevel_ByteArray[i]);
     checkSum += lightLevel_ByteArray[i];
   }
    Serial1.write((byte)0x7C); checkSum += 0x7C; // |
  //-----------------------------------------------------------------------------------------

   // Water Level
   memcpy(currentWaterLevel_ByteArray, &currentWaterLevel,4);
   for(int i = 0; i < 4; i++)
   {
     Serial1.write((byte)currentWaterLevel_ByteArray[i]);
     checkSum += currentWaterLevel_ByteArray[i];
   }
    Serial1.write((byte)0x7C); checkSum += 0x7C; // |
  //-----------------------------------------------------------------------------------------

  // Sensor RED led
  if(redLed)
  {
    Serial1.write((byte)0x31); checkSum += 0x31; // 1
  }
  else
  {
    Serial1.write((byte)0x30); checkSum += 0x30; // 0
  }

  Serial1.write((byte)0x7C); checkSum += 0x7C; // |
  //-----------------------------------------------------------------------------------------

  // UV leds
  if(uvLedsONOFF)
  {
    Serial1.write((byte)0x30); checkSum += 0x30; // 0
  }
  else
  {
    Serial1.write((byte)0x31); checkSum += 0x31; // 1
  }

  Serial1.write((byte)0x7C); checkSum += 0x7C; // |
  //-----------------------------------------------------------------------------------------

  // Water Pump
  if(buttPumpONOFF)
  {
    Serial1.write((byte)0x30); checkSum += 0x30; // 0
  }
  else
  {
    Serial1.write((byte)0x31); checkSum += 0x31; // 1
  }

  Serial1.write((byte)0x7C); checkSum += 0x7C; // |
  //-----------------------------------------------------------------------------------------

  // Float 1
  if(float1ONOFF)
  {
    Serial1.write((byte)0x31); checkSum += 0x31; // 1
  }
  else
  {
    Serial1.write((byte)0x30); checkSum += 0x30; // 0
  }

  Serial1.write((byte)0x7C); checkSum += 0x7C; // |
  //-----------------------------------------------------------------------------------------

  // Float 2
  if(float2ONOFF)
  {
    Serial1.write((byte)0x31); checkSum += 0x31; // 1
  }
  else
  {
    Serial1.write((byte)0x30); checkSum += 0x30; // 0
  }
  Serial1.write((byte)0x23); checkSum += 0x23; // #
 //-----------------------------------------------------------------------------------------

   Serial1.write((byte)0xFF - (checkSum & 0xFF));  // calc and send checksum
   checkSum = 0; // Reset checksum

//  long sum = 0x17 + 0xFF + 0xFF + 0xFF + 0xFE + 0x02 +'D' + '1' + value;
//  Serial11.write(0xFF - (sum & 0xFF), BYTE);  calc checksum
}


void ToggleLEDs()
{
  led = !led;
  digitalWrite(BoxRedLED, led);
  digitalWrite(BoxGreenLED, led);
  digitalWrite(BoxBlueLED, led);
}

void LampTest()
{
    for (uint8_t i = 0; i < 6; i++)
    {
        digitalWrite(BoxGreenLED,HIGH);
        delay(200);
        digitalWrite(BoxBlueLED,HIGH);
        delay(200);
        digitalWrite(BoxRedLED,HIGH);
        delay(200);
        digitalWrite(BoxGreenLED,LOW);
        delay(200);
        digitalWrite(BoxBlueLED,LOW);
        delay(200);
        digitalWrite(BoxRedLED,LOW);
        delay(400);
    }
        for (int q = 0; q < 4; q++)
            {
                digitalWrite(uvLEDs, UV_LEDS_ON);  // turn on uv leds
                delay(500);
                digitalWrite(uvLEDs, UV_LEDS_OFF);  // turn off uv leds
                delay(500);
            }

        digitalWrite(ButtPump, PUMP_ON);   /// ON
        delay(3000);
        digitalWrite(ButtPump, PUMP_OFF);   /// OFF
}

void TurnOnBlueLED(bool ONOFF)
{
    digitalWrite(BoxBlueLED, ONOFF);
}

void TurnOnGreenLED(bool ONOFF)
{
    digitalWrite(BoxGreenLED, ONOFF);
}

void TurnOnRedLED(bool ONOFF)
{
    digitalWrite(BoxRedLED, ONOFF);
}

void  DimFlashBlueLED( int Brightness, int length)
{
  if(Brightness < 1) {Brightness = 1;}
  for(int f = 0; f<length; f++)  // lenghth = 300 default
  {
    TurnOnBlueLED(HIGH);
    delayMicroseconds(Brightness);
    TurnOnBlueLED(LOW);
    delayMicroseconds(850);
  }
}

void  DimFlashGreenLED( int Brightness, int length)
{
  if(Brightness < 1) {Brightness = 1;}
  for(int g = 0; g<length; g++)
  {
    TurnOnGreenLED(HIGH);
    delayMicroseconds(Brightness);
    TurnOnGreenLED(LOW);
    delayMicroseconds(850);
  }
}

void  DimFlashRedLED( int Brightness, int length)
{
  if(Brightness < 1) {Brightness = 1;}
  for(int h = 0; h<length; h++)
  {
    TurnOnRedLED(HIGH);
    delayMicroseconds(Brightness);
    TurnOnRedLED(LOW);
    delayMicroseconds(850);
  }
}

void  DimFlashRedBlueLED(int Brightness, int length)
{
  if(Brightness < 1) {Brightness = 1;}
  for(int j = 0; j<length; j++)
  {
    digitalWrite(BoxRedLED,HIGH);  // if you want use the analogpins as digital than A0-> 14 ,A1->15,..etc
    digitalWrite(BoxBlueLED,HIGH);
    delayMicroseconds(Brightness);
    digitalWrite(BoxRedLED,LOW);
    digitalWrite(BoxBlueLED,LOW);
    delayMicroseconds(850);
  }
}

void  DimFlashRedGreenLED(int Brightness, int length)
{
  if(Brightness < 1) {Brightness = 1;}
  for(int k = 0; k<length; k++)
  {
    digitalWrite(BoxRedLED,HIGH);  // if you want use the analogpins as digital than A0-> 14 ,A1->15,..etc
    digitalWrite(BoxGreenLED,HIGH);
    delayMicroseconds(Brightness);
    digitalWrite(BoxRedLED,LOW);
    digitalWrite(BoxGreenLED,LOW);
    delayMicroseconds(850);
  }
}

void  DimFlashRedBlueGreenLED(int Brightness, int length)
{
  if(Brightness < 1) {Brightness = 1;}
  for(int m = 0; m<length; m++)
  {
    digitalWrite(BoxRedLED,HIGH);  // if you want use the analogpins as digital than A0-> 14 ,A1->15,..etc
    digitalWrite(BoxBlueLED,HIGH);
    digitalWrite(BoxGreenLED,HIGH);
    delayMicroseconds(Brightness);
    digitalWrite(BoxRedLED,LOW);
    digitalWrite(BoxBlueLED,LOW);
    digitalWrite(BoxGreenLED,LOW);
    delayMicroseconds(850);
  }
}

void  FreezingWaterLEDs()
{
  for(int n = 0; n<5; n++)
  {
     DimFlashRedLED(ledFlashBrightness,150);
     delay(150);
     DimFlashBlueLED(ledFlashBrightness,150);
     delay(150);
  }
}

bool GetDHT11Temperature()
{
      // Reading temperature or humidity takes about 250 milliseconds!
      // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
      dhtHumidity = dht.readHumidity();
      dhtTemperature = dht.readTemperature();   // Read temperature as Celsius (the default)

     if (isnan(dhtHumidity) || isnan(dhtTemperature)) {
        return false;
     }
     else {
         return true;
     }
}
