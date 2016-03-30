

/* Ranciolo Silvia Giga Mod.
    Full digital take over of Racilio Silvia V3 coffee machine with Arduino Uno and 4D systems touch screen interface.

    Designed for a full internal install utilizing modified original switches as low voltage inputs. The Arduino nano
    handles all the hardware sensing and machine control while the uLCD-43DT inteligent display module handles sensor
    readouts and programmable features.

    Featuring:
      Adjustable temperature control
      Temperature monitoring
      Pressure monitoring
      Brew timer
      Programable Pre infusion
      Programable automatic brew
      Low water indicator/safety

*/

//Libraries
#include "genieArduino.h"
#include <max6675.h>
#include <Wire.h>
#include "RTClib.h"
#include <EEPROM.h>


//Pinout Map
#define touchRX 0
#define touchTX 1
#define maxS0 2
#define maxCS 3
#define maxSCK 4
#define switchBrew 5
#define switchWater 6
#define switchSteam 7
#define ssrBoiler 8
#define ssrSolenoid 9
#define ssrPump 10
#define touchRST 11
#define pressureTrans A0
#define currentBoiler A1
#define currentPump A2

//Module Setup
Genie genie;
MAX6675 thermocouple(maxSCK, maxCS, maxS0);
RTC_DS1307 rtc;

//Initialize and reset global vars
bool switchBrewSt = 0;
bool switchBrewLast = 0;
bool switchSteamSt  = 0;
bool switchSteamLast = 0;
bool switchWaterSt = 0;
bool switchWaterLast = 0;


//Hardware Vars
int temp = 0;
int pressure = 0;
int boilerCurrent = 0;
int pumpCurrent = 0;

//Hardware State
bool boilerOn = 0;
bool pumpOn = 0;
bool solenoidOn = 0;

//Temprature settings
int brewTempSet = 105;
int brewBuffer = 2;
int brewHigh = 0;
int brewLow = 0;

int steamTempSet = 145;
int steamBuffer = 2;
int steamHigh = 0;
int steamLow = 0;

//Temprature Memory Addresses
#define brewTempSetMem 1
#define brewBufferMem 2

#define steamTempSetMem 3
#define steamBufferMem 4


//Timers
int brewTimer = 0;

unsigned long brewCount = 0;
const long secondInterval = 1000;

//Programs
bool brewManual = 0;                              //Brewing manualy

int preinfusionTime1 = 4;
int preinfusionDelay1 = 2;
int brewProgramTime1 = 20;

int preinfusionTime2 = 6;
int preinfusionDelay2 = 2;
int brewProgramTime2 = 18;

bool runBrew1 = 0;                                  //
bool brew1ProgOn = 0;
bool brew1ProgRun = 0;
int brew1ProgPhase = 0;                             //[0,1,2,3] [Not running, Pre-infusion, Delay, Brew]

bool runBrew2 = 0;
bool brew2ProgOn = 0;
bool brew2ProgRun = 0;
int brew2ProgPhase = 0;                             //[0,1,2,3] [Not running, Pre-infusion, Delay, Brew]

//Program Memory Addresses
#define preinfusionTime1Mem 5
#define preinfusionDelay1Mem 6
#define brewProgramTime1Mem 7

#define preinfusionTime2Mem 8
#define preinfusionDelay2Mem 9
#define brewProgramTime2Mem 10

//LCD
int menu = 0;                                       //[0,1,2] [Main, Prog, Temp]
int menuLast = 0;

//Time
int rtcMinuteLast = 0;
int rtcTimerLast = 0;




void setup() {

  //Comms setup
  Serial.begin(56000);
  genie.Begin(Serial);
  genie.AttachEventHandler(myGenieEventHandler);
  Wire.begin();
  rtc.begin();

  //Set Clock
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  //Let LCD reboot
  digitalWrite(touchRST, 1);
  delay(100);
  digitalWrite(touchRST, 0);
  delay(3500);

  //GPIO setup
  pinMode(switchBrew, INPUT);
  pinMode(switchWater, INPUT);
  pinMode(switchSteam, INPUT);
  pinMode(ssrBoiler, OUTPUT);
  pinMode(ssrSolenoid, OUTPUT);
  pinMode(ssrPump, OUTPUT);
  pinMode(touchRST, OUTPUT);
  pinMode(pressureTrans, INPUT);
  pinMode(currentBoiler, INPUT);
  pinMode(currentPump, INPUT);

  //Gauge Runup
  int gaugeSpool = 0;
  int gaugeRunup = 200;
  int gaugeRundown = 200;
  int tempSpool = 0;
  while (gaugeRunup > 0) {
    gaugeRunup--;
    gaugeSpool++;
    tempSpool = (gaugeSpool / 1.25);
    genie.WriteObject (GENIE_OBJ_COOL_GAUGE, 0X00, tempSpool);
    genie.WriteObject (GENIE_OBJ_COOL_GAUGE, 0X01, gaugeSpool);
    delay(20);
  }
  while (gaugeRundown > 0) {
    gaugeRundown--;
    gaugeSpool++;
    tempSpool = (gaugeSpool / 1.25);
    genie.WriteObject (GENIE_OBJ_COOL_GAUGE, 0X00, tempSpool);
    genie.WriteObject (GENIE_OBJ_COOL_GAUGE, 0X01, gaugeSpool);
    delay(20);
  }
  genie.WriteObject (GENIE_OBJ_COOL_GAUGE, 0X00, 0);
  genie.WriteObject (GENIE_OBJ_COOL_GAUGE, 0X01, 0);


  //Get Program Values From EEPROM
  brewTempSet = EEPROM.read(brewTempSetMem);
  brewBuffer = EEPROM.read(brewBufferMem);
  steamTempSet = EEPROM.read(steamTempSetMem);
  steamBuffer = EEPROM.read(steamBufferMem);

  preinfusionTime1 = EEPROM.read(preinfusionTime1Mem);
  preinfusionDelay1 = EEPROM.read(preinfusionDelay1Mem);
  brewProgramTime1 = EEPROM.read(brewProgramTime1Mem);

  preinfusionTime2 = EEPROM.read(preinfusionTime2Mem);
  preinfusionDelay2 = EEPROM.read(preinfusionDelay2Mem);
  brewProgramTime2 = EEPROM.read(brewProgramTime2Mem);

}

void loop () {
  //Read Temperature and Pressure
  temp = thermocouple.readCelsius();
  pressure = analogRead(pressureTrans);          //needs cal formula


  //Switch State Read
  switchBrewSt = digitalRead(switchBrew);
  switchSteamSt = digitalRead(switchSteam);
  switchSteamSt = digitalRead(switchSteam);


  //Temperature Control
  if (switchSteamSt = 1) {
    if ((temp > 5) && (temp <= steamLow)) {
      digitalWrite(ssrBoiler, HIGH);
      boilerOn = 1;
    }
    if (temp >= steamHigh) {
      digitalWrite(ssrBoiler, LOW);
      boilerOn = 0;
    }
    else {
      if ((temp > 5) && (temp <= brewLow)) {
        digitalWrite(ssrBoiler, HIGH);
        boilerOn = 1;
      }
      if (temp >= brewHigh) {
        digitalWrite(ssrBoiler, LOW);
        boilerOn = 0;
      }
    }
  }

  //LCD Temperature and Pressure Gauges
  if (menu = 1) {
    genie.WriteObject (GENIE_OBJ_COOL_GAUGE, 0X00, temp);
    genie.WriteObject (GENIE_OBJ_COOL_GAUGE, 0X01, pressure);
  }

  //Time
  DateTime now = rtc.now();
  if (menu == 0) {
    if (now.minute() != rtcMinuteLast) {
      genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x11, (now.hour(), DEC));
      genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x12, (now.minute(), DEC));
      rtcMinuteLast = now.second();
    }
  }

  //Manual Brew
  if ((brew1ProgOn == 0) && (brew2ProgOn == 0)) {
    if (switchBrewSt != switchBrewLast) {
      if (switchBrewSt = HIGH) {
        digitalWrite(ssrSolenoid, HIGH);
        digitalWrite(ssrPump, HIGH);
        digitalWrite(ssrSolenoid, HIGH);
        brewManual = 1;
        genie.WriteObject(GENIE_OBJ_FORM, 1, 0);
        menu = 0;
        brewTimer = 0;
      }
      else {
        digitalWrite(ssrSolenoid, LOW);
        digitalWrite(ssrPump, LOW);
        digitalWrite(ssrSolenoid, LOW);
        brewManual = 0;
      }
    }
  }

  //Manual Brew Timer
  if (bool brewManual = 1) {
    unsigned long currentTimer = millis();
    if (currentTimer - brewCount >= secondInterval) {
      brewCount = currentTimer; {
        brewTimer++;
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x00, brewTimer);
      }
    }
  }



  //Brew Programs
  //Brew1
  if (brew1ProgOn == 1) {
    if (switchBrewSt != switchBrewLast) {
      if (switchBrewSt = HIGH) {
        brewTimer = preinfusionTime1;
        brew1ProgPhase = 1;
        runBrew1 = 1;
      }
      else {
        digitalWrite(ssrSolenoid, LOW);
        digitalWrite(ssrPump, LOW);
        digitalWrite(ssrSolenoid, LOW);
        runBrew1 = 0;
      }
    }

    if (runBrew1 = 1) {
      switch (brew1ProgPhase) {
        case 1:                                                             //Preinfusion
          if (brewTimer > 0) {
            genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x00, brewTimer);
            unsigned long currentTimer = millis();
            if (currentTimer - brewCount >= secondInterval) {
              brewCount = currentTimer; {
                brewTimer--;
                genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x00, brewTimer);
              }
            }
            if (solenoidOn = 0) {
              digitalWrite (ssrSolenoid, 1);
              solenoidOn = 1;
            }
            if (pumpOn = 0) {
              digitalWrite (ssrPump, 1);
              pumpOn = 1;
            }
          }
          else {
            brew1ProgPhase = 2;
            brewTimer = preinfusionDelay1;
          }
          break;
        case 2:                                                             //Preinfusion Delay
          if (brewTimer > 0) {
            genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x00, brewTimer);
            unsigned long currentTimer = millis();
            if (currentTimer - brewCount >= secondInterval) {
              brewCount = currentTimer; {
                brewTimer--;
                genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x00, brewTimer);
              }
            }
            if (solenoidOn = 0) {
              digitalWrite (ssrSolenoid, 1);
              solenoidOn = 1;
            }
            if (pumpOn = 1) {
              digitalWrite (ssrPump, 0);
              pumpOn = 0;
            }
          }
          else {
            brew1ProgPhase = 3;
            brewTimer = brewProgramTime1;
          }
          break;
        case 3:                                                             //Brew
          if (brewTimer > 0) {
            genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x00, brewTimer);
            unsigned long currentTimer = millis();
            if (currentTimer - brewCount >= secondInterval) {
              brewCount = currentTimer; {
                brewTimer--;
                genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x00, brewTimer);
              }
            }
            if (solenoidOn = 0) {
              digitalWrite (ssrSolenoid, 1);
              solenoidOn = 1;
            }
            if (pumpOn = 0) {
              digitalWrite (ssrPump, 1);
              pumpOn = 1;
            }
          }
          else {
            brew1ProgPhase = 0;
            if (solenoidOn = 1) {
              digitalWrite (ssrSolenoid, 0);
              solenoidOn = 0;
            }
            if (pumpOn = 0) {
              digitalWrite (ssrPump, 0);
              pumpOn = 0;
            }
            break;
          }
      }
    }
  }

  //Brew1
  if (brew2ProgOn == 1) {
    if (switchBrewSt != switchBrewLast) {
      if (switchBrewSt = HIGH) {
        brewTimer = preinfusionTime2;
        brew2ProgPhase = 1;
        runBrew2 = 1;
      }
      else {
        digitalWrite(ssrSolenoid, LOW);
        digitalWrite(ssrPump, LOW);
        digitalWrite(ssrSolenoid, LOW);
        runBrew2 = 0;
        brew2ProgPhase = 0;
      }
    }

    if (runBrew2 = 1) {
      switch (brew2ProgPhase) {
        case 1:                                                             //Preinfusion
          if (brewTimer > 0) {
            genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x00, brewTimer);
            unsigned long currentTimer = millis();
            if (currentTimer - brewCount >= secondInterval) {
              brewCount = currentTimer; {
                brewTimer--;
                genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x00, brewTimer);
              }
            }
            if (solenoidOn = 0) {
              digitalWrite (ssrSolenoid, 1);
              solenoidOn = 1;
            }
            if (pumpOn = 0) {
              digitalWrite (ssrPump, 1);
              pumpOn = 1;
            }
          }
          else {
            brew1ProgPhase = 2;
            brewTimer = preinfusionDelay2;
          }
          break;
        case 2:                                                             //Preinfusion Delay
          if (brewTimer > 0) {
            genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x00, brewTimer);
            unsigned long currentTimer = millis();
            if (currentTimer - brewCount >= secondInterval) {
              brewCount = currentTimer; {
                brewTimer--;
                genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x00, brewTimer);
              }
            }
            if (solenoidOn = 0) {
              digitalWrite (ssrSolenoid, 1);
              solenoidOn = 1;
            }
            if (pumpOn = 1) {
              digitalWrite (ssrPump, 0);
              pumpOn = 0;
            }
          }
          else {
            brew2ProgPhase = 3;
            brewTimer = brewProgramTime2;
          }
          break;
        case 3:                                                             //Brew
          if (brewTimer > 0) {
            genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x00, brewTimer);
            unsigned long currentTimer = millis();
            if (currentTimer - brewCount >= secondInterval) {
              brewCount = currentTimer; {
                brewTimer--;
                genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x00, brewTimer);
              }
            }
            if (solenoidOn = 0) {
              digitalWrite (ssrSolenoid, 1);
              solenoidOn = 1;
            }
            if (pumpOn = 0) {
              digitalWrite (ssrPump, 1);
              pumpOn = 1;
            }
          }
          else {
            brew2ProgPhase = 0;
            if (solenoidOn = 1) {
              digitalWrite (ssrSolenoid, 0);
              solenoidOn = 0;
            }
            if (pumpOn = 0) {
              digitalWrite (ssrPump, 0);
              pumpOn = 0;
            }
            break;
          }
      }
    }
  }

  //Switch State Monitoring
  switchBrewLast = digitalRead(switchBrew);
  switchSteamLast = digitalRead(switchSteam);
  switchSteamLast = digitalRead(switchSteam);
}


void myGenieEventHandler(void)  {

  genieFrame Event;
  //  genieDequeueEvent(&Event);                                   //wont compile, should work??


  if (Event.reportObject.cmd == GENIE_REPORT_EVENT)                                   //If the cmd received is from a Reported Event
  {
    if (Event.reportObject.object == GENIE_OBJ_USERBUTTON) {                           // If the Reported Message was from a USERBUTTON
      if (Event.reportObject.index == 0 || 4 || 8) {                                  // If it was a Brew 1 button
        brew1ProgOn = genie.GetEventData(&Event);                                     //Avtivate or deactivate the brew 1 program
        genie.WriteObject(GENIE_OBJ_USERBUTTON, 0x01, 0);
        genie.WriteObject(GENIE_OBJ_USERBUTTON, 0x05, 0);
        genie.WriteObject(GENIE_OBJ_USERBUTTON, 0x09, 0);
        brew2ProgOn = 0;
      }
      if (Event.reportObject.index == 1 || 5 || 9) {                                  // If it was a Brew 2 button
        brew2ProgOn = genie.GetEventData(&Event);                                     //Avtivate or deactivate the brew 2 program
        genie.WriteObject(GENIE_OBJ_USERBUTTON, 0x00, 0);
        genie.WriteObject(GENIE_OBJ_USERBUTTON, 0x04, 0);
        genie.WriteObject(GENIE_OBJ_USERBUTTON, 0x08, 0);
        brew1ProgOn = 0;
      }
      if (Event.reportObject.index == 2 || 10) {                                      // If it was a Prog button
        genie.WriteObject(GENIE_OBJ_FORM, 1, 0);                                       // Change to Form 1, Program page
        menu = 1;
      }
      if (Event.reportObject.index == 3 || 7) {                                      // If it was a Temp button
        genie.WriteObject(GENIE_OBJ_FORM, 2, 0);                                       // Change to Form 2, Temperature page
        menu = 2;
      }
    }

    //Brew Program Adjust Menu
    if (menu = 1) {                                                                  //If we are on the program page
      if (Event.reportObject.index == 6) {                                      // If it was the Prog button
        genie.WriteObject(GENIE_OBJ_FORM, 0, 0);                                       // Change to Form 1, Program page
        menu = 0;
      }
      if (Event.reportObject.object == GENIE_OBJ_4DBUTTON) {                          //If it was a 4d button

        //Brew 1
        if (Event.reportObject.index == 8) {                                        // If it was the preinfusion 1 minus button
          preinfusionTime1--;                                                           //Decrement preinfusion
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x01, preinfusionTime1);               // Display new value
          EEPROM.write(preinfusionTime1, preinfusionTime1Mem);
        }
        if (Event.reportObject.index == 9) {                                          // If it was the preinfusion 1 plus button
          preinfusionTime1++;                                                           //inccrement preinfusion
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x01, preinfusionTime1);               // Display new value
          EEPROM.write(preinfusionTime1, preinfusionTime1Mem);
        }
        if (Event.reportObject.index == 10) {                                        // If it was the preinfusion 1 minus button
          preinfusionDelay1--;                                                           //Decrement Delay 1
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x06, preinfusionDelay1);               // Display new value
          EEPROM.write(preinfusionDelay1, preinfusionDelay1Mem);
        }
        if (Event.reportObject.index == 11) {                                          // If it was the preinfusion plus button
          preinfusionDelay1++;                                                           //Increment Delay 1
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x06, preinfusionDelay1);               // Display new value
          EEPROM.write(preinfusionDelay1, preinfusionDelay1Mem);
        }
        if (Event.reportObject.index == 14) {                                        // If it was the preinfusion 1 minus button
          brewProgramTime1--;                                                           //Decrement brew 1 Program Time
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x07, brewProgramTime1);               // Display new value
          EEPROM.write(brewProgramTime1 , brewProgramTime1Mem);
        }
        if (Event.reportObject.index == 15) {                                          // If it was the preinfusion plus button
          brewProgramTime1++;                                                           //Increment brew 1 Program Time
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x07, brewProgramTime1);               // Display new value
          EEPROM.write(brewProgramTime1 , brewProgramTime1Mem);
        }

        //Brew 2
        if (Event.reportObject.index == 12) {                                        // If it was the preinfusion 2 minus button
          preinfusionTime2--;                                                           //Decrement preinfusion 2
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x08, preinfusionTime2);               // Display new value
          EEPROM.write(preinfusionTime2 , preinfusionTime2Mem );
        }
        if (Event.reportObject.index == 13) {                                          // If it was the preinfusion 2 plus button
          preinfusionTime2++;                                                           //inccrement preinfusion 2
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x08, preinfusionTime2);               // Display new value
          EEPROM.write(preinfusionTime2 , preinfusionTime2Mem );
        }
        if (Event.reportObject.index == 16) {                                        // If it was the preinfusion 2 minus button
          preinfusionDelay2--;                                                           //Decrement Delay 2
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x09, preinfusionDelay2);               // Display new value
          EEPROM.write(preinfusionDelay2 , preinfusionDelay2Mem );
        }
        if (Event.reportObject.index == 17) {                                          // If it was the preinfusion plus button
          preinfusionDelay2++;                                                           //Increment Delay 2
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x09, preinfusionDelay2);               // Display new value
          EEPROM.write(preinfusionDelay2 , preinfusionDelay2Mem );
        }
        if (Event.reportObject.index == 18) {                                        // If it was the preinfusion 1 minus button
          brewProgramTime2--;                                                           //Decrement brew 2 Program Time
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x0A, brewProgramTime2);               // Display new value
          EEPROM.write( brewProgramTime2, brewProgramTime2Mem);
        }
        if (Event.reportObject.index == 19) {                                          // If it was the preinfusion plus button
          brewProgramTime2++;                                                           //Increment brew 2 Program Time
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x0A, brewProgramTime2);               // Display new value
          EEPROM.write( brewProgramTime2, brewProgramTime2Mem);
        }
      }
    }
    //Temperature Adjust Menu
    if (menu = 2) {                                                                  //If we are on the temperature page
      if (Event.reportObject.index == 11) {                                      // If it was the Temp button
        genie.WriteObject(GENIE_OBJ_FORM, 0, 0);                                       // Change to Form 1, Program page
        menu = 0;
      }

      //Brew Temperatue
      if (Event.reportObject.index == 0) {                                        // If it was the brew temp minus button
        brewTempSet--;                                                                 //Decrement brew temp
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x04, brewTempSet);               // Display new value
        EEPROM.write(brewTempSet , brewTempSetMem );
      }
      if (Event.reportObject.index == 1) {                                          // If it was the brew temp plus button
        brewTempSet++;                                                               //Increment brew temp
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x04, brewTempSet);               // Display new value
        EEPROM.write(brewTempSet , brewTempSetMem );
      }
      if (Event.reportObject.index == 2) {                                        // If it was the brew temp minus button
        brewBuffer--;                                                                 //Decrement brew buffer
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x03, brewBuffer);               // Display new value
        EEPROM.write(brewBuffer , brewBufferMem );
      }
      if (Event.reportObject.index == 3) {                                          // If it was the brew temp plus button
        brewBuffer++;                                                               //Increment brew buffer
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x03, brewBuffer);               // Display new value
        EEPROM.write(brewBuffer , brewBufferMem );
      }

      //Steam Temperature
      if (Event.reportObject.index == 4) {                                        // If it was the steam temp minus button
        steamTempSet--;                                                                 //Decrement brew temp
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x05, steamTempSet);               // Display new value
        EEPROM.write(steamTempSet , steamTempSetMem );
      }
      if (Event.reportObject.index == 5) {                                          // If it was the steam temp plus button
        steamTempSet++;                                                               //Increment brew temp
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x05, steamTempSet);               // Display new value
        EEPROM.write(steamTempSet , steamTempSetMem );
      }
      if (Event.reportObject.index == 6) {                                        // If it was the steam temp minus button
        steamBuffer--;                                                                 //Decrement steam buffer
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x02, steamBuffer);               // Display new value
        EEPROM.write(steamBuffer , steamBufferMem );
      }
      if (Event.reportObject.index == 7) {                                          // If it was the steam temp plus button
        steamBuffer++;                                                               //Increment steam buffer
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x024, steamBuffer);               // Display new value
        EEPROM.write(steamBuffer , steamBufferMem );
      }
    }
  }
}
