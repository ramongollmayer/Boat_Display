/**************************************************************************
 This is an example for our Monochrome OLEDs based on SSD1306 drivers

 Pick one up today in the adafruit shop!
 ------> http://www.adafruit.com/category/63_98

 This example is for a 128x64 pixel display using I2C to communicate
 3 pins are required to interface (two I2C and one reset).

 Adafruit invests time and resources providing this open
 source code, please support Adafruit and open-source
 hardware by purchasing products from Adafruit!

 Written by Limor Fried/Ladyada for Adafruit Industries,
 with contributions from the open source community.
 BSD license, check license.txt for more information
 All text above, and the splash screen below must be
 included in any redistribution.
 **************************************************************************/
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <TimeLib.h>
#include <Arduino.h>
#include <SD.h>

#define IncrementTurn1 27   //define Pin for Incremental pin 1
#define IncrementTurn2 25  //define Pin for Incremental pin 2
#define IncrementButton 26   //define Pin for Incremental Push
#define userMenuePages 10 // define how much menu pages are inside the user Menue
#define SDA1 21
#define SCL1 22
#define SDA2 33
#define SCL2 32
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define SCREEN_WIDTH2 128
#define SCREEN_HEIGHT2 32
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define SCREEN_ADDRESS2 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define NUMFLAKES     10 // Number of snowflakes in the animation example
#define bitmap_Satelite_height 16
#define bitmap_Satelite_width 16
#define bitmap_Satelite_FAIL_height 16
#define bitmap_Satelite_FAIL_width 16
#define bitmap_SDCardOK_height 16
#define bitmap_SDCardOK_width 16
#define bitmap_SDCardFAIL_height 16
#define bitmap_SDCardFAIL_width 16
#define SEALEVELPRESSURE_HPA (1013.25)
#define time_offset   3600  // define a clock offset of 3600 seconds (1 hour) ==> UTC + 1
#define CS_PIN 5  /** The clock select pin for the SD card module */
#define Serial0RX 3
#define Serial0TX 1

TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2Cone, OLED_RESET);
Adafruit_SSD1306 display2(SCREEN_WIDTH, SCREEN_HEIGHT, &I2Cone, OLED_RESET);
Adafruit_SSD1306 display3(SCREEN_WIDTH2, SCREEN_HEIGHT2, &I2Ctwo, OLED_RESET);

//SD CARD Variables
bool SDSetupOkFlag = false;
File file;
const char *skipperFile = "/SkipperList.txt";

//Logbook variables
int currentSkipper = 1;
int SkippersInList = 1;
String currentSkipperName;

// BME280 object
Adafruit_BMP280 bmp(&I2Cone); // I2C

//Inviromental Variables
float pressureInPascal = 0.0;
float pressureInBar = 0.0;
float temperature = 0.0;

static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;
//gps values
double lastGpsPositionLng = 0.0;
double lastGpsPositionLat = 0.0;
double actualTripDistance = 0.0;
double getGpsLat = 0.0;
double getGpsLng = 0.0;
double getGpsSpeed = 0.0;
int getGpsERROR = 0;
bool gpsFirstMessurement = true;
//Fuel consumption
double fuelConsumption = 0.0;
double fuelConsumptionMultiplier = 0.7;

// Define the hardware serial port
HardwareSerial gpsSerial(2); // Use gpsSerial on ESP32 for GPS module
HardwareSerial Serial0(0);

//ISR for Incremental Sensor
bool userTurnIncrement1 = false; //Flag if Flank on pin 1 changes
bool userTurnIncrement2 = false; //Flag if Flank on pin 2 changes
bool userPushIncrement = false; //Flag if Push accurs
bool userInMenue = false; //checks if user is in menue
int userMenuPosition = 0; //Position for Menu controled via Increment
int userMenuPositionSub_1 = 0;
int userMenuePositionVal = 0;
bool userMenuPositionSub_1_Flag = false;
bool userMenuePositionVal_Flag = false;
int displayChoserOled1 = 0;
int displayChoserOled2 = 1;

void IRAM_ATTR IncrementTurn1ISR()
{
  userTurnIncrement1 = true;
}
void IRAM_ATTR IncrementTurn2ISR()
{
  userTurnIncrement2 = true;
}
void IRAM_ATTR IncrementPush()
{
  userPushIncrement = true;
}

//Bitmaps for Symboles on Top Screen

static const unsigned char PROGMEM bitmap_Satelite[] = {
 B00000000, B00010000,
 B00000000, B00101000,
 B00000000, B01000100,
 B00000000, B10101010,
 B00001100, B10010001,
 B00001110, B10101010,
 B00000111, B01000100,
 B00000011, B10111000,
 B00011101, B11000000,
 B00100010, B11100000,
 B01010101, B01100001,
 B10001001, B00010101,
 B01010101, B00000101,
 B00100010, B00011101,
 B00010100, B00000001,
 B00001000, B00111111,
};

static const unsigned char PROGMEM bitmap_Satelite_FAIL[] = {
 B00000000, B00010000,
 B00000000, B00101000,
 B00000000, B01000100,
 B00000000, B10101010,
 B00001100, B10010001,
 B00001110, B10101010,
 B00000111, B01000100,
 B00000011, B10111000,
 B00011101, B11000000,
 B00100010, B11100000,
 B01010101, B01100000,
 B10001001, B00000000,
 B01010101, B00000000,
 B00100010, B00000000,
 B00010100, B00000000,
 B00001000, B00000000,
};

static const unsigned char PROGMEM bitmap_SDCardOK[] = {
 B00011111, B11111111,
 B00010101, B01010101,
 B00010101, B01010101,
 B00010101, B01010101,
 B00111111, B11111111,
 B00111001, B11001111,
 B01110110, B11010111,
 B01111011, B11010111,
 B01111101, B11010111,
 B00110110, B11010111,
 B00111001, B11001111,
 B01111111, B11111111,
 B01111000, B10101111,
 B01111010, B10011111,
 B01111000, B10101111,
 B01111111, B11111111,
};

static const unsigned char PROGMEM bitmap_SDCardFAIL[] = {
 B00011111, B11111111,
 B00010101, B01010101,
 B00010101, B01010101,
 B00010101, B01010101,
 B00111111, B11111111,
 B00111001, B11001111,
 B01110110, B11010111,
 B01111011, B11010111,
 B01111101, B11010111,
 B00110110, B11010111,
 B00111001, B11001111,
 B01111111, B11111111,
 B01100110, B11010111,
 B01101101, B01010111,
 B01100100, B01010111,
 B01101101, B01010001,
};

// variable definitions for TimeLib
char Time[]  = "TIME: 00:00:00";
char Date[]  = "DATE: 00-00-2000";
byte last_second, Second, Minute, Hour, Day, Month;
int Year;
/****************************************************************Setup***********************************************************/
void setup() {
  Serial.begin(9600);
  //Sim Module
  Serial0.begin(9600,SERIAL_8N1,Serial0RX,Serial0TX); //Serial for SIM800L
  test_sim800_module();

  if (!SD.begin(CS_PIN)) {
    SDSetupOkFlag = false;
  }else{
    SDSetupOkFlag = true;
  }

  I2Cone.begin(SDA1,SCL1,100000); // SDA pin 21, SCL pin 22 TTGO TQ
  I2Ctwo.begin(SDA2,SCL2,400000); // SDA2 pin 17, SCL2 pin 16 

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    for(;;); // Don't proceed, loop forever
  }
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display2.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS2)) {
    for(;;); // Don't proceed, loop forever
  }
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display3.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS2)) {
    for(;;); // Don't proceed, loop forever
  }
  
  // Clear the buffer
  display.clearDisplay();
  display2.clearDisplay();
  display3.clearDisplay();
  display.display();
  display2.display();
  display3.display();
  delay(2000);

  gpsSerial.begin(GPSBaud); //Start GPS Comunication

  //Setup Incrementalwheel
  pinMode(IncrementTurn1, INPUT_PULLUP);
  pinMode(IncrementTurn2, INPUT_PULLUP);
  pinMode(IncrementButton, INPUT_PULLUP);
  attachInterrupt(IncrementTurn1, IncrementTurn1ISR, CHANGE);
  attachInterrupt(IncrementTurn2, IncrementTurn2ISR, CHANGE);
  attachInterrupt(IncrementButton, IncrementPush, RISING); 
  
bmp.begin(0x76);

bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  
  startscreen();      // Show Startscreen
}
/******************************************************************Main**************************************************************/
void loop() {
  //Sim
  updateSerial();

 if(last_second != gps.time.second())  // if time has changed
      {
        last_second = gps.time.second();
 
        // set current UTC time
        setTime(Hour, Minute, Second, Day, Month, Year);
        // add the offset to get local time
        adjustTime(time_offset);

        // update time array
        Time[12] = second() / 10 + '0';
        Time[13] = second() % 10 + '0';
        Time[9]  = minute() / 10 + '0';
        Time[10] = minute() % 10 + '0';
        Time[6]  = hour()   / 10 + '0';
        Time[7]  = hour()   % 10 + '0';
 
        // update date array
        Date[14] = (year()  / 10) % 10 + '0';
        Date[15] =  year()  % 10 + '0';
        Date[9]  =  month() / 10 + '0';
        Date[10] =  month() % 10 + '0';
        Date[6]  =  day()   / 10 + '0';
        Date[7]  =  day()   % 10 + '0';
      }
//This code controls the upper display and also the Encoder Button and also navigating thrught the menu 
  if(userPushIncrement == true || userInMenue == true){
    if(userPushIncrement == true && userInMenue == false){
      userInMenue = true;
      userPushIncrement = false;
      userMenuPosition = 0;
    }
//Check if encoder is rotated left or rigt, userMenuPosition goes up for right and down for left
//It also checks if the max Page count has an overflow and adjust it to 0 or max Page count
    if(userTurnIncrement1 == true){
      userTurnIncrement1 = false;
      if(digitalRead(IncrementTurn1) == HIGH) {     //Rising Flank
        if(digitalRead(IncrementTurn2) == HIGH){
          if(userMenuPositionSub_1_Flag == false){
            userMenuPosition = userMenuPosition + 1;
          }else{
            if(userMenuePositionVal_Flag == false){
              userMenuPositionSub_1 ++;
            }else{
              userMenuePositionVal++;
            }
          }
        }
        else{
          if(userMenuPositionSub_1_Flag == false){
            userMenuPosition = userMenuPosition - 1;
          }else{
            if(userMenuePositionVal_Flag == false){
              userMenuPositionSub_1 --;
            }else{
              userMenuePositionVal--;
            }
          }
        }
      }
      else{                                       //Falling Flank
        if(digitalRead(IncrementTurn2) == HIGH){
          if(userMenuPositionSub_1_Flag == false){
            userMenuPosition = userMenuPosition - 1;
          }else{
            if(userMenuePositionVal_Flag == false){
              userMenuPositionSub_1 --;
            }else{
              userMenuePositionVal--;
            }
          }
        }
        else{
          if(userMenuPositionSub_1_Flag == false){
            userMenuPosition = userMenuPosition + 1;
          }else{
            if(userMenuePositionVal_Flag == false){
              userMenuPositionSub_1 ++;
            }else{
              userMenuePositionVal++;
            }
          }
        }
      }
    }

    if(userTurnIncrement2 == true){
      userTurnIncrement2 = false;
      if(digitalRead(IncrementTurn2) == HIGH){    //Rising Flank
        if(digitalRead(IncrementTurn1) == HIGH){
          if(userMenuPositionSub_1_Flag == false){
            userMenuPosition = userMenuPosition - 1;
          }else{
            if(userMenuePositionVal_Flag == false){
              userMenuPositionSub_1 --;
            }else{
              userMenuePositionVal--;
            }
          }
        }
        else{
          if(userMenuPositionSub_1_Flag == false){
            userMenuPosition = userMenuPosition + 1;
          }else{
            if(userMenuePositionVal_Flag == false){
              userMenuPositionSub_1 ++;
            }else{
              userMenuePositionVal++;
            }
          }
        }
      }
      else{                                     //Falling Flank
        if(digitalRead(IncrementTurn1) == HIGH){
          if(userMenuPositionSub_1_Flag == false){
            userMenuPosition = userMenuPosition + 1;
          }else{
            if(userMenuePositionVal_Flag == false){
              userMenuPositionSub_1 ++;
            }else{
              userMenuePositionVal++;
            }
          }
        }
        else{
          if(userMenuPositionSub_1_Flag == false){
            userMenuPosition = userMenuPosition - 1;
          }else{
            if(userMenuePositionVal_Flag == false){
              userMenuPositionSub_1 --;
            }else{
              userMenuePositionVal--;
            }
          }
        }
      }
    }
    //Check if Menu Position has an overflolw and adjust it
    if(userMenuPosition < 0)
    {
      userMenuPosition = userMenuePages;
    }
    else if(userMenuPosition > userMenuePages)
    {
      userMenuPosition = 0;
    }
//Hear is the menue constructed
    switch(userMenuPosition)
    {
//*************************************************************   Menu Speed   *************************************************************
      case 0:
      if(userMenuPositionSub_1 < 1 && userMenuPositionSub_1_Flag == true){
        userMenuPositionSub_1 = 3;
      }else if(userMenuPositionSub_1 > 3 && userMenuPositionSub_1_Flag == true){
        userMenuPositionSub_1 = 1;
      }
      switch(userMenuPositionSub_1){  // create sub menue
        case 0:
        display3.clearDisplay();
        display3.setTextSize(2);
        display3.setTextColor(SSD1306_WHITE);
        display3.setCursor((SCREEN_WIDTH2 - (5 * 12)) / 2, display3.height()/2);
        display3.println("Speed");
        display3.display();

        if(userPushIncrement == true){
          userPushIncrement = false;
          userMenuPositionSub_1 = 1;
          userMenuPositionSub_1_Flag = true;
        }
        break;

        case 1:
        display3.clearDisplay();
        display3.setTextSize(2);
        display3.setTextColor(SSD1306_WHITE);
        display3.setCursor((SCREEN_WIDTH2 - (9 * 12)) / 2, 0);
        display3.println("Speed on:");
        display3.setCursor((SCREEN_WIDTH2 - (9 * 12)) / 2, display3.height() / 2);
        display3.println("Left oled");
        display3.display();
        if(userPushIncrement == true){
          userPushIncrement = false;
          displayChoserOled1 = 0;
        }
        break;

        case 2:
        display3.clearDisplay();
        display3.setTextSize(2);
        display3.setTextColor(SSD1306_WHITE);
        display3.setCursor((SCREEN_WIDTH2 - (9 * 12)) / 2, 0);
        display3.println("Speed on:");
        display3.setCursor((SCREEN_WIDTH2 - (10 * 12)) / 2, display3.height() / 2);
        display3.println("Right oled");
        display3.display();
        if(userPushIncrement == true){
          userPushIncrement = false;
          displayChoserOled2 = 0;
        }
        break;

        case 3:
        display3.clearDisplay();
        display3.setTextSize(2);
        display3.setTextColor(SSD1306_WHITE);
        display3.setCursor((SCREEN_WIDTH2 - (4 * 12)) / 2, display3.height()/2);
        display3.println("Exit");
        display3.display();
        if(userPushIncrement == true){
          userPushIncrement = false;
          userMenuPositionSub_1 = 0;
          userMenuPositionSub_1_Flag = false;
        }
        break;
      }

      break;
//*************************************************************   Menu Trip   *************************************************************
      case 1:
      if(userMenuPositionSub_1 < 1 && userMenuPositionSub_1_Flag == true){
        userMenuPositionSub_1 = 4;
      }else if(userMenuPositionSub_1 > 4 && userMenuPositionSub_1_Flag == true){
        userMenuPositionSub_1 = 1;
      }
      switch(userMenuPositionSub_1){  // create sub menue
        case 0:
        display3.clearDisplay();
        display3.setTextSize(2);
        display3.setTextColor(SSD1306_WHITE);
        display3.setCursor((SCREEN_WIDTH2 - (4 * 12)) / 2, display3.height()/2);
        display3.println("Trip");
        display3.display();

        if(userPushIncrement == true){
          userPushIncrement = false;
          userMenuPositionSub_1 = 1;
          userMenuPositionSub_1_Flag = true;
        }
        break;

        case 1:
        display3.clearDisplay();
        display3.setTextSize(2);
        display3.setTextColor(SSD1306_WHITE);
        display3.setCursor((SCREEN_WIDTH2 - (8 * 12)) / 2, 0);
        display3.println("Trip on:");
        display3.setCursor((SCREEN_WIDTH2 - (9 * 12)) / 2, display3.height() / 2);
        display3.println("Left oled");
        display3.display();
        if(userPushIncrement == true){
          userPushIncrement = false;
          displayChoserOled1 = 1;
        }
        break;

        case 2:
        display3.clearDisplay();
        display3.setTextSize(2);
        display3.setTextColor(SSD1306_WHITE);
        display3.setCursor((SCREEN_WIDTH2 - (8 * 12)) / 2, 0);
        display3.println("Trip on:");
        display3.setCursor((SCREEN_WIDTH2 - (10 * 12)) / 2, display3.height() / 2);
        display3.println("Right oled");
        display3.display();
        if(userPushIncrement == true){
          userPushIncrement = false;
          displayChoserOled2 = 1;
        }
        break;

        case 3:
        display3.clearDisplay();
        display3.setTextSize(2);
        display3.setTextColor(SSD1306_WHITE);
        display3.setCursor((SCREEN_WIDTH2 - (5 * 12)) / 2, 0);
        display3.println("Reset");
        display3.setCursor((SCREEN_WIDTH2 - (4 * 12)) / 2, display3.height() / 2);
        display3.println("Trip");
        display3.display();
        break;

        case 4:
        display3.clearDisplay();
        display3.setTextSize(2);
        display3.setTextColor(SSD1306_WHITE);
        display3.setCursor((SCREEN_WIDTH2 - (4 * 12)) / 2, display3.height()/2);
        display3.println("Exit");
        display3.display();
        if(userPushIncrement == true){
          userPushIncrement = false;
          userMenuPositionSub_1 = 0;
          userMenuPositionSub_1_Flag = false;
        }
        break;
      }
      break;
//*************************************************************   Menu fuel gauge   *************************************************************
      case 2:
      if(userMenuPositionSub_1 < 1 && userMenuPositionSub_1_Flag == true){
        userMenuPositionSub_1 = 4;
      }else if(userMenuPositionSub_1 > 4 && userMenuPositionSub_1_Flag == true){
        userMenuPositionSub_1 = 1;
      }
      switch(userMenuPositionSub_1){  // create sub menue
        case 0:
        display3.clearDisplay();
        display3.setTextSize(2);
        display3.setTextColor(SSD1306_WHITE);
        display3.setCursor((SCREEN_WIDTH2 - (10 * 12)) / 2, display3.height()/2);
        display3.println("Fuel Gauge");
        display3.display();

        if(userPushIncrement == true){
          userPushIncrement = false;
          userMenuPositionSub_1 = 1;
          userMenuPositionSub_1_Flag = true;
        }
        break;

        case 1:
        display3.clearDisplay();
        display3.setTextSize(2);
        display3.setTextColor(SSD1306_WHITE);
        display3.setCursor((SCREEN_WIDTH2 - (8 * 12)) / 2, 0);
        display3.println("Fuel on:");
        display3.setCursor((SCREEN_WIDTH2 - (9 * 12)) / 2, display3.height() / 2);
        display3.println("Left oled");
        display3.display();
        if(userPushIncrement == true){
          userPushIncrement = false;
          displayChoserOled1 = 2;
        }
        break;

        case 2:
        display3.clearDisplay();
        display3.setTextSize(2);
        display3.setTextColor(SSD1306_WHITE);
        display3.setCursor((SCREEN_WIDTH2 - (8 * 12)) / 2, 0);
        display3.println("Fuel on:");
        display3.setCursor((SCREEN_WIDTH2 - (10 * 12)) / 2, display3.height() / 2);
        display3.println("Right oled");
        display3.display();
        if(userPushIncrement == true){
          userPushIncrement = false;
          displayChoserOled2 = 2;
        }
        break;

        case 3:
        display3.clearDisplay();
        display3.setTextSize(2);
        display3.setTextColor(SSD1306_WHITE);
        display3.setCursor((SCREEN_WIDTH2 - (10 * 12)) / 2, 0);
        display3.println("Fuel Multi:");
        display3.setCursor((SCREEN_WIDTH2 - (10 * 12)) / 2, display3.height() / 2);
        display3.print(fuelConsumptionMultiplier,1);
        display3.display();
        if(userPushIncrement == true && userMenuePositionVal_Flag == false){
          userPushIncrement = false;
          userMenuePositionVal_Flag = true;   
        }
        if(userPushIncrement == true && userMenuePositionVal_Flag == true){
          userPushIncrement = false;
          userMenuePositionVal_Flag = false;
        }
        if(userMenuePositionVal_Flag == true){
          fuelConsumptionMultiplier = fuelConsumptionMultiplier + userMenuePositionVal * 0.1;
          userMenuePositionVal = 0;
        }
        break;


        case 4:
        display3.clearDisplay();
        display3.setTextSize(2);
        display3.setTextColor(SSD1306_WHITE);
        display3.setCursor((SCREEN_WIDTH2 - (4 * 12)) / 2, display3.height()/2);
        display3.println("Exit");
        display3.display();
        if(userPushIncrement == true){
          userPushIncrement = false;
          userMenuPositionSub_1 = 0;
          userMenuPositionSub_1_Flag = false;
        }
        break;
      }

      break;
//*************************************************************   Menu Temp & Hum   *************************************************************
      case 3:
      if(userMenuPositionSub_1 < 1 && userMenuPositionSub_1_Flag == true){
        userMenuPositionSub_1 = 3;
      }else if(userMenuPositionSub_1 > 3 && userMenuPositionSub_1_Flag == true){
        userMenuPositionSub_1 = 1;
      }
      switch(userMenuPositionSub_1){  // create sub menue
        case 0:
        display3.clearDisplay();
        display3.setTextSize(2);
        display3.setTextColor(SSD1306_WHITE);
        display3.setCursor((SCREEN_WIDTH2 - (8 * 12)) / 2, display3.height()/2);
        display3.println("Temp&Hum");
        display3.display();

        if(userPushIncrement == true){
          userPushIncrement = false;
          userMenuPositionSub_1 = 1;
          userMenuPositionSub_1_Flag = true;
        }
        break;

        case 1:
        display3.clearDisplay();
        display3.setTextSize(2);
        display3.setTextColor(SSD1306_WHITE);
        display3.setCursor((SCREEN_WIDTH2 - (9 * 12)) / 2, 0);
        display3.println("Temp&Hum:");
        display3.setCursor((SCREEN_WIDTH2 - (9 * 12)) / 2, display3.height() / 2);
        display3.println("Left oled");
        display3.display();
        if(userPushIncrement == true){
          userPushIncrement = false;
          displayChoserOled1 = 3;
        }
        break;

        case 2:
        display3.clearDisplay();
        display3.setTextSize(2);
        display3.setTextColor(SSD1306_WHITE);
        display3.setCursor((SCREEN_WIDTH2 - (9 * 12)) / 2, 0);
        display3.println("Temp&Hum:");
        display3.setCursor((SCREEN_WIDTH2 - (10 * 12)) / 2, display3.height() / 2);
        display3.println("Right oled");
        display3.display();
        if(userPushIncrement == true){
          userPushIncrement = false;
          displayChoserOled2 = 3;
        }
        break;

        case 3:
        display3.clearDisplay();
        display3.setTextSize(2);
        display3.setTextColor(SSD1306_WHITE);
        display3.setCursor((SCREEN_WIDTH2 - (4 * 12)) / 2, display3.height()/2);
        display3.println("Exit");
        display3.display();
        if(userPushIncrement == true){
          userPushIncrement = false;
          userMenuPositionSub_1 = 0;
          userMenuPositionSub_1_Flag = false;
        }
        break;
      }

      break;
//*************************************************************   Logbook   *************************************************************
      case 4:
      if(userMenuPositionSub_1 < 1 && userMenuPositionSub_1_Flag == true){
        userMenuPositionSub_1 = 3;
      }else if(userMenuPositionSub_1 > 3 && userMenuPositionSub_1_Flag == true){
        userMenuPositionSub_1 = 1;
      }
      switch(userMenuPositionSub_1){  // create sub menue
        case 0:
        display3.clearDisplay();
        display3.setTextSize(2);
        display3.setTextColor(SSD1306_WHITE);
        display3.setCursor((SCREEN_WIDTH2 - (7 * 12)) / 2, display3.height()/2);
        display3.println("Logbook");
        display3.display();

        if(userPushIncrement == true){
          userPushIncrement = false;
          userMenuPositionSub_1 = 1;
          userMenuPositionSub_1_Flag = true;
        }
        break;

        case 1:
        file = SD.open(skipperFile, FILE_READ);
        if (!file) {
        Serial.println("Error opening txt");
        }
        file.seek(0);
        for (int i = 1; i < currentSkipper; i++) {
        // Skip lines until the desired line is reached
        file.readStringUntil('\n');
        }
        currentSkipperName = file.readStringUntil('\n');
        file.close(); // Close the file after counting

        display3.clearDisplay();
        display3.setTextSize(2);
        display3.setTextColor(SSD1306_WHITE);
        display3.setCursor((SCREEN_WIDTH2 - (7 * 12)) / 2, 0);
        display3.println("Choose:");
        display3.setCursor((SCREEN_WIDTH2 - (9 * 12)) / 2, display3.height() / 2);
        //display3.println("Skipper ");
        //display3.println(currentSkipper);
        display3.println(currentSkipperName);
        display3.display();
        if(userPushIncrement == true && userMenuePositionVal_Flag == false){
          userPushIncrement = false;
          userMenuePositionVal_Flag = true;   
        }
        if(userPushIncrement == true && userMenuePositionVal_Flag == true){
          userPushIncrement = false;
          userMenuePositionVal_Flag = false;
        }
        if(userMenuePositionVal_Flag == true){  
          file = SD.open(skipperFile, FILE_READ); //Read how many lines are in the file
          int lineCount = 0;
          while (file.available()) {
            if (file.read() == '\n') {
              lineCount++;
            }
          }
          file.close(); // Close the file after counting
          SkippersInList = lineCount; //write the line count inside the variable

          currentSkipper = currentSkipper + userMenuePositionVal;
          if(currentSkipper < 1){
            currentSkipper = SkippersInList;
          }
          if(currentSkipper > SkippersInList){
            currentSkipper = 1;
          }
          userMenuePositionVal = 0;


        }
        break;

        case 2:
        display3.clearDisplay();
        display3.setTextSize(2);
        display3.setTextColor(SSD1306_WHITE);
        display3.setCursor((SCREEN_WIDTH2 - (9 * 12)) / 2, 0);
        display3.println("Temp&Hum:");
        display3.setCursor((SCREEN_WIDTH2 - (10 * 12)) / 2, display3.height() / 2);
        display3.println("Right oled");
        display3.display();
        if(userPushIncrement == true){
          userPushIncrement = false;
          displayChoserOled2 = 3;
        }
        break;

        case 3:
        display3.clearDisplay();
        display3.setTextSize(2);
        display3.setTextColor(SSD1306_WHITE);
        display3.setCursor((SCREEN_WIDTH2 - (4 * 12)) / 2, display3.height()/2);
        display3.println("Exit");
        display3.display();
        if(userPushIncrement == true){
          userPushIncrement = false;
          userMenuPositionSub_1 = 0;
          userMenuPositionSub_1_Flag = false;
        }
        break;
      }

      break;
//*************************************************************   Menu 5   *************************************************************
      case 5:
      display3.clearDisplay();
      display3.setTextSize(2);
      display3.setTextColor(SSD1306_WHITE);
      display3.setCursor((SCREEN_WIDTH2 - (6 * 12)) / 2, display3.height()/2);
      display3.println("Page 5");
      display3.display();

      break;
//*************************************************************   Menu 6   *************************************************************
      case 6:
      display3.clearDisplay();
      display3.setTextSize(2);
      display3.setTextColor(SSD1306_WHITE);
      display3.setCursor((SCREEN_WIDTH2 - (6 * 12)) / 2, display3.height()/2);
      display3.println("Page 6");
      display3.display();

      break;
//*************************************************************   Menu 7   *************************************************************
      case 7:
      display3.clearDisplay();
      display3.setTextSize(2);
      display3.setTextColor(SSD1306_WHITE);
      display3.setCursor((SCREEN_WIDTH2 - (6 * 12)) / 2, display3.height()/2);
      display3.println("Page 7");
      display3.display();

      break;
//*************************************************************   Menu 8   *************************************************************
      case 8:
      display3.clearDisplay();
      display3.setTextSize(2);
      display3.setTextColor(SSD1306_WHITE);
      display3.setCursor((SCREEN_WIDTH2 - (6 * 12)) / 2, display3.height()/2);
      display3.println("Page 8");
      display3.display();

      break;
//*************************************************************   Menu 9   *************************************************************
      case 9:
      display3.clearDisplay();
      display3.setTextSize(2);
      display3.setTextColor(SSD1306_WHITE);
      display3.setCursor((SCREEN_WIDTH2 - (6 * 12)) / 2, display3.height()/2);
      display3.println("Page 9");
      display3.display();

      break;
//*************************************************************   Menu 10   *************************************************************
      case 10:
      display3.clearDisplay();
      display3.setTextSize(2);
      display3.setTextColor(SSD1306_WHITE);
      display3.setCursor((SCREEN_WIDTH2 - (4 * 12)) / 2, display3.height()/2);
      display3.println("Exit");
      display3.display();

      if(userPushIncrement == true){
        userPushIncrement = false;
        userInMenue = false;
      }
      break;
    }
//*************************************************************   default   *************************************************************
  }else{  //Display Standard Values
    display3.clearDisplay();
    display3.setTextSize(1);
    display3.setTextColor(SSD1306_WHITE);
    display3.setCursor((SCREEN_WIDTH2 - (14 * 6)) / 2, display3.height()/1.5);
    display3.println(Time);
    if(gps.location.isValid()){
      display3.drawBitmap(0, 0, bitmap_Satelite, bitmap_Satelite_width, bitmap_Satelite_height, WHITE);
    }else{
      display3.drawBitmap(0, 0, bitmap_Satelite_FAIL, bitmap_Satelite_FAIL_width, bitmap_Satelite_FAIL_height, WHITE);
    }
    if(SDSetupOkFlag == true){
      display3.drawBitmap(20, 0, bitmap_SDCardOK, bitmap_SDCardOK_width, bitmap_SDCardOK_height, WHITE);
    }else{
      display3.drawBitmap(20, 0, bitmap_SDCardFAIL, bitmap_SDCardFAIL_width, bitmap_SDCardFAIL_height, WHITE);
    }
    display3.display();
  }
//**********************************Handle Gps Speed, Position and Trip**********************************************************
//Also create the Display that shows the Data on the oleds 
  getGPSLonLatSpeed(getGpsLat, getGpsLng, getGpsSpeed, getGpsERROR);

//Create the Display that shows the Speed
  if(displayChoserOled1 == 0){
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor((SCREEN_WIDTH - (5 * 12)) / 2, 0);  // Center the text vertically
    display.print("Speed");
    display.setTextColor(WHITE);
    display.setTextSize(4);
    display.setCursor(0, (SCREEN_HEIGHT / 4) * 3 - 12);  // Center the text vertically
    display.print(getGpsSpeed, 1);  // Display the speed value with one decimal place
    display.setTextSize(1);
    display.setCursor(SCREEN_WIDTH - 4 * 6, ((SCREEN_HEIGHT / 4) * 3 - 12) + 16 + 2);
    display.print("Km/h");

    display.display();
  }

  if(displayChoserOled2 == 0){
    display2.clearDisplay();
    display2.setTextSize(2);
    display2.setTextColor(SSD1306_WHITE);
    display2.setCursor((SCREEN_WIDTH - (5 * 12)) / 2, 0);  // Center the text vertically
    display2.print("Speed");
    display2.setTextColor(WHITE);
    display2.setTextSize(4);
    display2.setCursor(0, (SCREEN_HEIGHT / 4) * 3 - 12);  // Center the text vertically
    display2.print(getGpsSpeed, 1);  // Display the speed value with one decimal place
    display2.setTextSize(1);
    display2.setCursor(SCREEN_WIDTH - 4 * 6, ((SCREEN_HEIGHT / 4) * 3 - 12) + 16 + 2);
    display2.print("Km/h");

    display2.display();
  }

  
//Calculate Distance between last point and actuale point
  if(gps.location.isValid()){
    if(gpsFirstMessurement == true){
      gpsFirstMessurement = false;
      lastGpsPositionLng = gps.location.lng();
      lastGpsPositionLat = gps.location.lat();
    }
    double distanceKmToLastPoint =
    (double)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      lastGpsPositionLat,
      lastGpsPositionLng) / 1000;

    actualTripDistance = actualTripDistance + distanceKmToLastPoint;
    lastGpsPositionLng = gps.location.lng();
    lastGpsPositionLat = gps.location.lat();
  }
    if(displayChoserOled1 == 1){
      display.clearDisplay();
      display.setTextSize(2);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor((SCREEN_WIDTH - (4 * 12)) / 2, 0);  // Center the text vertically
      display.print("Trip");
      display.setTextColor(WHITE);
      display.setTextSize(4);
      display.setCursor(0, (SCREEN_HEIGHT / 4) * 3 - 12);  // Center the text vertically
      display.print(actualTripDistance, 2);  // Display the speed value with one decimal place
      display.setTextSize(1);
      display.setCursor(SCREEN_WIDTH - 2 * 6, ((SCREEN_HEIGHT / 4) * 3 - 12) + 16 + 2);
      display.print("Km");

      display.display();
    }

    if(displayChoserOled2 == 1){
      display2.clearDisplay();
      display2.setTextSize(2);
      display2.setTextColor(SSD1306_WHITE);
      display2.setCursor((SCREEN_WIDTH - (4 * 12)) / 2, 0);  // Center the text vertically
      display2.print("Trip");
      display2.setTextColor(WHITE);
      display2.setTextSize(4);
      display2.setCursor(0, (SCREEN_HEIGHT / 4) * 3 - 12);  // Center the text vertically
      display2.print(actualTripDistance, 2);  // Display the speed value with one decimal place
      display2.setTextSize(1);
      display2.setCursor(SCREEN_WIDTH - 2 * 6, ((SCREEN_HEIGHT / 4) * 3 - 12) + 16 + 2);
      display2.print("Km");

      display2.display();
    }
    
//Calculate Fule Consumption from Trip
  fuelConsumption = actualTripDistance * fuelConsumptionMultiplier;
  if(displayChoserOled1 == 2){
      display.clearDisplay();
      display.setTextSize(2);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor((SCREEN_WIDTH - (9 * 12)) / 2, 0);  // Center the text vertically
      display.print("Fuel Used");
      display.setTextColor(WHITE);
      display.setTextSize(4);
      display.setCursor(0, (SCREEN_HEIGHT / 4) * 3 - 12);  // Center the text vertically
      display.print(fuelConsumption, 1);  // Display the speed value with one decimal place
      display.setTextSize(1);
      display.setCursor(SCREEN_WIDTH - 1 * 6, ((SCREEN_HEIGHT / 4) * 3 - 12) + 16 + 2);
      display.print("L");

      display.display();
    }

  if(displayChoserOled2 == 2){
      display2.clearDisplay();
      display2.setTextSize(2);
      display2.setTextColor(SSD1306_WHITE);
      display2.setCursor((SCREEN_WIDTH - (9 * 12)) / 2, 0);  // Center the text vertically
      display2.print("Fuel Used");
      display2.setTextColor(WHITE);
      display2.setTextSize(4);
      display2.setCursor(0, (SCREEN_HEIGHT / 4) * 3 - 12);  // Center the text vertically
      display2.print(fuelConsumption, 1);  // Display the speed value with one decimal place
      display2.setTextSize(1);
      display2.setCursor(SCREEN_WIDTH - 1 * 6, ((SCREEN_HEIGHT / 4) * 3 - 12) + 16 + 2);
      display2.print("L");

      display2.display();
    }
//****************************************** Inviromental Mesurements (Temp,Hum,Pres)*************************************************
  pressureInPascal = (bmp.readPressure() / 100);
  pressureInBar = pressureInPascal / 100000;
  temperature = bmp.readTemperature();

  if(displayChoserOled1 == 3){
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor((SCREEN_WIDTH - (10 * 12)) / 2, 0);  // Center the text vertically
    display.print("Temp.&Hum.");

    display.setTextColor(WHITE);
    display.setTextSize(2);
    display.setCursor(0, (SCREEN_HEIGHT / 6) * 3 - 12);  // Center the text vertically
    display.print("T:");
    display.print(temperature, 1);  // Display the first number with one decimal place
    display.setTextSize(1);
    display.print("C");

    display.setTextSize(2);
    display.setCursor(0, (SCREEN_HEIGHT / 6) * 3 + 16);  // Position for the second number
    display.print("p:");
    display.print(pressureInPascal, 1);  // Display the second number with one decimal place
    display.setTextSize(1);
    display.print("hPa");

    display.display();
  }

  if(displayChoserOled2 == 3){
    display2.clearDisplay();
    display2.setTextSize(2);
    display2.setTextColor(SSD1306_WHITE);
    display2.setCursor((SCREEN_WIDTH - (10 * 12)) / 2, 0);  // Center the text vertically
    display2.print("Temp.&Hum.");

    display2.setTextColor(WHITE);
    display2.setTextSize(2);
    display2.setCursor(0, (SCREEN_HEIGHT / 6) * 3 - 12);  // Center the text vertically
    display2.print("T:");
    display2.print(temperature, 1);  // Display the first number with one decimal place
    display.setTextSize(1);
    display2.print("C");

    display.setTextSize(2);
    display2.setCursor(0, (SCREEN_HEIGHT / 6) * 3 + 16);  // Position for the second number
    display2.print("p:");
    display2.print(pressureInPascal, 1);  // Display the second number with one decimal place
    display.setTextSize(1);
    display2.print("hPa");

    display2.display();
    }
/****************************************************************Writing Data to SD-Card*********************************************/

}

/****************************************************************Functions***********************************************************/
void startscreen(void) {
  
  display.clearDisplay();
  display2.clearDisplay();
  display3.clearDisplay();

  for(int16_t i=0; i<display.height()/2; i+=2) {
    display.drawRect(i, i, display.width()-2*i, display.height()-2*i, SSD1306_WHITE);
    display.display(); // Update screen with each newly-drawn rectangle
    delay(1);
  }


  for(int16_t i=0; i<display2.height()/2; i+=2) {
    display2.drawRect(i, i, display2.width()-2*i, display2.height()-2*i, SSD1306_WHITE);
    display2.display(); // Update screen with each newly-drawn rectangle
    delay(1);
  }


 for(int16_t i=0; i<display3.height()/2; i+=2) {
    display3.drawRect(i, i, display3.width()-2*i, display3.height()-2*i, SSD1306_WHITE);
    display3.display(); // Update screen with each newly-drawn rectangle
    delay(1);
  }
  delay(1000);
  display3.clearDisplay();
  display3.setTextSize(2);
  display3.setTextColor(SSD1306_WHITE);
  display3.setCursor(0, display3.height()/2);
  display3.println("Welcome!");
  display3.display();
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, display.height()/2);
  display.println("A Boat for");
  display.display();
  display2.clearDisplay();
  display2.setTextSize(2);
  display2.setTextColor(SSD1306_WHITE);
  display2.setCursor(0, display2.height()/2);
  display2.println("Boys&Girls");
  display2.display();
  delay(5000);
  display.clearDisplay();
  display.display();
  display2.clearDisplay();
  display2.display();
  display3.clearDisplay();
  display3.display();
}
// Read Speed and LAT / LON from GPS module
void getGPSLonLatSpeed(double& gpsLat, double& gpsLng, double& gpsSpeed, int& gpsERROR) {
  unsigned long start = millis();
  while (gpsSerial.available() > 0 && (millis() - 2000) < start) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        gpsLat = gps.location.lat();
        gpsLng = gps.location.lng();
      }
      else {
        gpsERROR = 1;
      }

      if (gps.speed.isValid()) {
        gpsSpeed = gps.speed.kmph(); // Get speed in kilometers per hour
        if(gpsSpeed >= 100.0 || gpsSpeed <= 1){
          gpsSpeed = 00.0;
        }
      }
      else {
        gpsERROR = 2;
      }
            // get time from GPS module
      if (gps.time.isValid())
      {
        Minute = gps.time.minute();
        Second = gps.time.second();
        Hour   = gps.time.hour();
      }
 
      // get date drom GPS module
      if (gps.date.isValid())
      {
        Day   = gps.date.day();
        Month = gps.date.month();
        Year  = gps.date.year();
      }

    if (millis() > 5000 && gps.charsProcessed() < 10) {
      gpsERROR = 3;
    }
  }
}
}
//*************************SIM MODULE*******************************************
void test_sim800_module()
{
  Serial0.println("AT");
  updateSerial();
  Serial.println();
  Serial0.println("AT+CSQ");
  updateSerial();
  Serial0.println("AT+CCID");
  updateSerial();
  Serial0.println("AT+CREG?");
  updateSerial();
  Serial0.println("ATI");
  updateSerial();
  Serial0.println("AT+CBC");
  updateSerial();
}
void updateSerial()
{
  delay(500);
  while (Serial.available())
  {
    Serial0.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while (Serial0.available())
  {
    Serial.write(Serial0.read());//Forward what Software Serial received to Serial Port
  }
}

void send_SMS()
{
  Serial0.println("AT+CMGF=1"); // Configuring TEXT mode
  updateSerial();
  Serial0.println("AT+CMGS=\"+436504443147\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
  updateSerial();
  Serial0.print("Circuit Digest"); //text content
  updateSerial();
Serial.println();
  Serial.println("Message Sent");
  Serial0.write(26);
}
