/*---------------------------------------------------------------------------------------------------------------------*\
  ||            ___    _   _  __  __  __   __ __   __  _       ___  __ __  _ __ __  _       __   __  __    _             ||
  ||           / o | .' \ / |/ /,'_/ / /  / // /  / /,' \    ,' _/ / // / /// // /.' \    ,'_/  / /,'_/  .' \            ||
  ||          /  ,' / o // || // /_ / /_ / // /_ / // o |   _\ `. / // /_| V // // o /   / /_n / // /_n / o /            ||
  ||         /_/`_\/_n_//_/|_/ |__//___//_//___//_/ |_,'   /___,'/_//___/|_,'/_//_n_/    |__,'/_/ |__,'/_n_/             ||
  ||---------------------------------------------------------------------------------------------------------------------||
  || Ranciolo Silvia Giga Mod.                                                                                           ||
  ||                                                                                                                     ||
  || Full digital take over of Racilio Silvia V3 coffee machine with Arduino Nano and 4D systems touch screen interface. ||
  ||                                                                                                                     ||
  || Designed for a completely internal integrated install utilizing modified original switches as low voltage inputs.   ||
  || The Arduino nano handles all the hardware sensing and machine control while the uLCD-43DT inteligent display        ||
  ||  module handles sensor readouts and provides a GUI for programmable features.                                       ||
  ||                                                                                                                     ||
  || Featuring:                                                                                                          ||
  ||  * Adjustable temperature control with software PID                                                                 ||
  ||  * Temperature monitoring                                                                                           ||
  ||  * Pressure monitoring                                                                                              ||
  ||  * Brew timer                                                                                                       ||
  ||  * Programable Pre infusion                                                                                         ||
  ||  * Programable automatic brew                                                                                       ||
  ||  * Low water indicator/safety                                                                                       ||
  ||  * Real time clock display                                                                                          ||
  ||  * Automatic backflush program                                                                                      ||
  ||---------------------------------------------------------------------------------------------------------------------||
  \*---------------------------------------------------------------------------------------------------------------------*/

//Libraries
#include <genieArduino.h>
#include <SPI.h>
#include <Thermocouple.h>
#include <Average.h>
#include <Wire.h>
#include "RTClib.h"
#include <EEPROMex.h>
#include <EEPROMVar.h>
#include <PID_v1.h>

//Pinout Map
#define touchTX 0
#define touchRX 1
#define touchRST 4
#define switchBrew 2
#define switchWater 3
#define switchSteam 5
#define ssrBoiler 6
#define ssrSolenoid 7
#define ssrPump 8
#define lowWater 9
#define maxCS 10
#define maxSO 12
#define maxSCK 13

#define pressureTrans A0
#define currentBoiler A1
#define currentPump A2

//Initialize and reset global vars
bool switchBrewSt = 0;
bool switchBrewLast = 0;
bool switchSteamSt  = 0;
bool switchSteamLast = 0;
bool switchWaterSt = 0;
bool switchWaterLast = 0;
bool lowWaterSt = 0;

//Hardware Vars
float temp = 0;
float pressure = 0;
float pressureRaw = 0;
float pressureCalc = 0;
Average<float> pressureAVG(40);

//Hardware State
bool boilerOn = 0;
bool pumpOn = 0;
bool solenoidOn = 0;

//Temprature settings
int brewTempSet = 105;
int brewBuffer = 30;
int steamTempSet = 138;
int steamBuffer = 2;

//PID Variables
double Setpoint;
double Input;
double Output;
double aggKp = 50;
double aggKi = 0;
double aggKd = 0;
double consKp = 30;
double consKi = 0;
double consKd = 0;
unsigned long WindowSize = 500;
unsigned long windowStartTime = 0;

//Temprature Memory Addresses
#define brewTempSetMem 0
#define brewBufferMem 4
#define steamTempSetMem 6
#define steamBufferMem 10
#define aggKpMem 12
#define aggKiMem 16
#define aggKdMem 20
#define consKpMem 24
#define consKiMem 28
#define consKdMem 32
#define WindowSizeMem 36

//Timers
int brewTimer = 0;

unsigned long brewCount = 0;
unsigned long backflushCount = 0;
const long secondInterval = 1000;

unsigned long tempCount = 0;
const long tempInterval = 300;

unsigned long graphCount = 0;
const long graphInterval = 100;

//Programs
bool brewManual = 0;                              //Brewing manualy

int preinfusionTime1 = 4;
int preinfusionDelay1 = 4;
int brewProgramTime1 = 20;

int preinfusionTime2 = 6;
int preinfusionDelay2 = 4;
int brewProgramTime2 = 18;

bool runBrew1 = 0;
bool brew1ProgOn = 0;
bool brew1ProgRun = 0;
int brew1ProgPhase = 0;                             //[0,1,2,3] [Not running, Pre-infusion, Delay, Brew]

bool runBrew2 = 0;
bool brew2ProgOn = 0;
bool brew2ProgRun = 0;
int brew2ProgPhase = 0;                             //[0,1,2,3] [Not running, Pre-infusion, Delay, Brew]

//Program Memory Addresses
#define preinfusionTime1Mem 40
#define preinfusionDelay1Mem 42
#define brewProgramTime1Mem 44

#define preinfusionTime2Mem 46
#define preinfusionDelay2Mem 48
#define brewProgramTime2Mem 50

//LCD
int menu = 0;                                       //[0,1,2,3,4,5] [Main, Prog, Temp, Graph, PID, Backflush]
int menuLast = 0;

//Time
int rtcMinuteLast = 0;
int rtcTimerLast = 0;

//Backflush
int backflushState = 0;                              //[0,1,2,3] [disabled, reset, run, stop]
int backflushCycleSet = 10;
int backflushCurrentCycle = 0;
int backflushTimer = 0;
int backflushOnTime = 5;
int backflushDelay = 5;
bool backflushPosition = 1;                         // [0, 1] [delay cycle, on cycle]

#define backflushCycleMem 52
#define backflushOnTimeMem 54
#define backflushDelayMem 56

//Module Setup
Genie genie;
Thermocouple tc1 = Thermocouple(maxCS);
RTC_DS1307 rtc;
PID tempPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);


void setup() {
  //Comms setup
  Serial.begin(115200);
  genie.Begin(Serial);
  genie.AttachEventHandler(myGenieEventHandler);
  Wire.begin();
  rtc.begin();
  tempPID.SetMode(AUTOMATIC);
  tempPID.SetOutputLimits(0, WindowSize);

  //Set Clock on Upload (run once)
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  //Let LCD reboot
  digitalWrite(touchRST, 0);
  delay(100);
  digitalWrite(touchRST, 1);
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
  int gaugeRunup = 25;
  int gaugeRundown = 25;
  int tempSpool = 0;
  while (gaugeRunup > 0) {
    gaugeRunup--;
    gaugeSpool = (gaugeSpool + 8);
    tempSpool = (gaugeSpool / 1.25);
    genie.WriteObject (GENIE_OBJ_COOL_GAUGE, 0X00, tempSpool);
    genie.WriteObject (GENIE_OBJ_COOL_GAUGE, 0X01, gaugeSpool);
  }
  while (gaugeRundown > 0) {
    gaugeRundown--;
    gaugeSpool = (gaugeSpool - 8);
    tempSpool = (gaugeSpool / 1.25);
    genie.WriteObject (GENIE_OBJ_COOL_GAUGE, 0X00, tempSpool);
    genie.WriteObject (GENIE_OBJ_COOL_GAUGE, 0X01, gaugeSpool);
  }
  genie.WriteObject (GENIE_OBJ_COOL_GAUGE, 0X00, 0);
  genie.WriteObject (GENIE_OBJ_COOL_GAUGE, 0X01, 0);

  //Initial Setup of EEPROM values (Run Once)
/*  EEPROM.writeDouble(brewTempSetMem , brewTempSet);
  delay (500);
  EEPROM.writeInt(brewBufferMem , brewBuffer);
  delay (500);
  EEPROM.writeDouble(steamTempSetMem , steamTempSet);
  delay (500);
  EEPROM.writeInt(steamBufferMem, steamBuffer);
  delay (500);
  EEPROM.writeDouble(aggKpMem, aggKp);
  delay (500);
  EEPROM.writeDouble(aggKiMem, aggKi);
  delay (500);
  EEPROM.writeDouble(aggKdMem, aggKd);
  delay (500);
  EEPROM.writeDouble(consKpMem, consKp);
  delay (500);
  EEPROM.writeDouble(consKiMem, consKi);
  delay (500);
  EEPROM.writeDouble(consKdMem, consKd);
  delay (500);
  EEPROM.writeLong(WindowSizeMem, WindowSize);
  delay (500);

  EEPROM.writeInt(preinfusionTime1Mem, preinfusionTime1);
  delay (500);
  EEPROM.writeInt(preinfusionDelay1Mem , preinfusionDelay1);
  delay (500);
  EEPROM.writeInt(brewProgramTime1Mem , brewProgramTime1);
  delay (500);
  EEPROM.writeInt(preinfusionTime2Mem , preinfusionTime2);
  delay (500);
  EEPROM.writeInt(preinfusionDelay2Mem , preinfusionDelay2);
  delay (500);
  EEPROM.writeInt(brewProgramTime2Mem , brewProgramTime2);
  delay (500);

  EEPROM.writeInt(backflushCycleMem, backflushCurrentCycle);
  delay (500);
  EEPROM.writeInt(backflushOnTimeMem, backflushOnTime);
  delay (500);
  EEPROM.writeInt(backflushDelayMem, backflushDelay);
  delay (500);
*/

  //Get Program Values From EEPROM
  //Temp settings
  brewTempSet = EEPROM.readDouble(brewTempSetMem);
  brewBuffer = EEPROM.readInt(brewBufferMem);
  steamTempSet = EEPROM.readDouble(steamTempSetMem);
  steamBuffer = EEPROM.readInt(steamBufferMem);
  //PID settings
  aggKp = EEPROM.readDouble(aggKpMem);
  aggKi = EEPROM.readDouble(aggKiMem);
  aggKd = EEPROM.readDouble(aggKdMem);
  consKp = EEPROM.readDouble(consKpMem);
  consKi = EEPROM.readDouble(consKiMem);
  consKd = EEPROM.readDouble(consKdMem);
  WindowSize = EEPROM.readLong(WindowSizeMem);
  //Program Settings
  preinfusionTime1 = EEPROM.readInt(preinfusionTime1Mem);
  preinfusionDelay1 = EEPROM.readInt(preinfusionDelay1Mem);
  brewProgramTime1 = EEPROM.readInt(brewProgramTime1Mem);
  preinfusionTime2 = EEPROM.readInt(preinfusionTime2Mem);
  preinfusionDelay2 = EEPROM.readInt(preinfusionDelay2Mem);
  brewProgramTime2 = EEPROM.readInt(brewProgramTime2Mem);
  //Backflush Settings
  backflushCurrentCycle = EEPROM.readInt(backflushCycleMem);
  backflushOnTime = EEPROM.readInt(backflushOnTimeMem);
  backflushDelay = EEPROM.readInt(backflushDelayMem);
}

void loop () {
  //Read Temperature
  unsigned long tempTimer = millis();
  if (tempTimer - tempCount >= tempInterval) {
    tempCount = tempTimer; {
      temp = (tc1.readC());
    }
  }

  //Read and calculate pressure
  pressureRaw = (analogRead(pressureTrans));
  pressureCalc = ((pressureRaw - 102) / 4.1);
  pressureAVG.push(pressureCalc);
  pressure = (pressureAVG.mean());
  if (pressure < 0) {
    pressure = 0;
  }

  //Switch State Read
  switchBrewSt = digitalRead(switchBrew);
  switchWaterSt = digitalRead(switchWater );
  switchSteamSt = digitalRead(switchSteam);
  lowWaterSt = digitalRead(lowWater);

  //Temperature Control
  if (switchSteamSt == 1) {
    Setpoint = steamTempSet;
    if (temp > 5) {                                                                                                          //safety check, thermostat fault returns negative value
      if (temp < (steamTempSet - steamBuffer)) {
        digitalWrite(ssrBoiler, HIGH);
        boilerOn = 1;
      }
      if (temp > steamTempSet) {
        digitalWrite(ssrBoiler, LOW);
        boilerOn = 0;
      }
    }
  }
  if (switchSteamSt == 0 ) {
    if (temp > 5) {                                                                                                           //safety check, thermostat fault returns negative value
      Setpoint = brewTempSet;
      Input = temp;
      double gap = abs(Setpoint - Input);
      if ((gap < brewBuffer) && (pumpOn == 0)) {                                                                               //if the machine is idle and the temperature is close to setpoint
        tempPID.SetTunings(consKp, consKi, consKd);                                                                           //use conservative PID constants
      }
      else {
        tempPID.SetTunings(aggKp, aggKi, aggKd);                                                                              //else use aggressive PID constants
      }
      tempPID.Compute();
      if (millis() - windowStartTime > WindowSize)  {                                                                         //time to shift the Relay Window
        windowStartTime = millis();
      }
      if (Output > millis() - windowStartTime) {
        if (temp < Setpoint) {
          digitalWrite(ssrBoiler, HIGH);
          boilerOn = 1;
        }
        else {
          digitalWrite(ssrBoiler, LOW);
          boilerOn = 0;
        }
      }
      else {
        digitalWrite(ssrBoiler, LOW);
        boilerOn = 0;
      }
    }
  }
  //LCD Temperature and Pressure Gauges
  if (menu == 0) {
    genie.WriteObject (GENIE_OBJ_COOL_GAUGE, 0X00, temp);
    genie.WriteObject (GENIE_OBJ_COOL_GAUGE, 0X01, pressure);
  }

  //LCD Temperature and Pressure Graph
  unsigned long graphTimer = millis();
  if (graphTimer - graphCount >= graphInterval) {
    graphCount = graphTimer; {
      genie.WriteObject (GENIE_OBJ_SCOPE, 0X00, pressure);
      genie.WriteObject (GENIE_OBJ_SCOPE, 0X00, Setpoint);
      genie.WriteObject (GENIE_OBJ_SCOPE, 0X00, temp);
    }
  }
  if (menu == 3) {
    genie.WriteObject (GENIE_OBJ_LED_DIGITS, 0X0D, temp);
    genie.WriteObject (GENIE_OBJ_LED_DIGITS, 0x1A, Setpoint);
    genie.WriteObject (GENIE_OBJ_LED_DIGITS, 0x0E, pressure);
  }

  //Low Water Indicator
  if (lowWaterSt == 0) {
    genie.WriteObject(GENIE_OBJ_USER_LED, 0x00, 0x01);
  }
  else {
    genie.WriteObject(GENIE_OBJ_USER_LED, 0x00, 0x00);
    if (switchBrew == 0) {
      digitalWrite(ssrPump, LOW);
      digitalWrite(ssrSolenoid, LOW);
      brewManual = 0;
    }
  }

  //Time
  DateTime now = rtc.now();
  if (menu == 0) {
    if (now.minute() != rtcMinuteLast) {
      int currentHour = ((int) now.hour());
      int currentMinute = ((int) now.minute());
      if (currentHour > 12) {
        currentHour -= 12;
      }
      genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x0B, currentHour);
      genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x0C, currentMinute);
      rtcMinuteLast = now.minute();
    }
  }

  //Manual Brew
  if (lowWaterSt == 1) {
    if ((brew1ProgOn == 0) && (brew2ProgOn == 0)) {
      if (switchBrewSt != switchBrewLast) {
        if (switchBrewSt == HIGH) {
          digitalWrite(ssrSolenoid, HIGH);
          digitalWrite(ssrPump, HIGH);
          digitalWrite(ssrSolenoid, HIGH);
          brewManual = 1;
          menu = 0;
          brewTimer = 0;
          if (menu != 0) {
            genie.WriteObject(GENIE_OBJ_FORM, 0, 0);
          }
        }
        else {
          digitalWrite(ssrSolenoid, LOW);
          digitalWrite(ssrPump, LOW);
          digitalWrite(ssrSolenoid, LOW);
          brewManual = 0;
        }
      }
    }
  }

  //Manual Brew Timer
  if (brewManual == 1) {
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
    if (lowWaterSt == 1) {
      if (switchBrewSt != switchBrewLast) {
        if (switchBrewSt == HIGH) {
          brewTimer = preinfusionTime1;
          brew1ProgPhase = 1;
          runBrew1 = 1;
          if (menu != 0) {
            genie.WriteObject(GENIE_OBJ_FORM, 0, 0);
          }
        }
        else {
          digitalWrite(ssrSolenoid, LOW);
          digitalWrite(ssrPump, LOW);
          digitalWrite(ssrSolenoid, LOW);
          runBrew1 = 0;
        }
      }
    }

    if (runBrew1 == 1) {
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
            digitalWrite (ssrSolenoid, 1);
            solenoidOn = 1;
            digitalWrite (ssrPump, 1);
            pumpOn = 1;
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
              digitalWrite (ssrSolenoid, 1);
              solenoidOn = 1;
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
            digitalWrite (ssrSolenoid, 1);
            solenoidOn = 1;
            digitalWrite (ssrPump, 1);
            pumpOn = 1;
          }
          else {
            brew1ProgPhase == 0;
            digitalWrite (ssrSolenoid, 0);
            solenoidOn = 0;
            digitalWrite (ssrPump, 0);
            pumpOn = 0;
          }
          break;
      }
    }
  }

  //Brew2
  if (brew2ProgOn == 1) {
    if (lowWaterSt == 1) {
      if (switchBrewSt != switchBrewLast) {
        if (switchBrewSt == HIGH) {
          brewTimer = preinfusionTime2;
          brew2ProgPhase = 1;
          runBrew2 = 1;
          if (menu != 0) {
            genie.WriteObject(GENIE_OBJ_FORM, 0, 0);
          }
        }
        else {
          digitalWrite(ssrSolenoid, LOW);
          digitalWrite(ssrPump, LOW);
          digitalWrite(ssrSolenoid, LOW);
          runBrew2 = 0;
          brew2ProgPhase = 0;
        }
      }
    }

    if (runBrew2 == 1) {
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
            digitalWrite (ssrSolenoid, 1);
            solenoidOn = 1;
            digitalWrite (ssrPump, 1);
            pumpOn = 1;
          }
          else {
            brew2ProgPhase = 2;
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
            digitalWrite (ssrSolenoid, 1);
            solenoidOn = 1;
            digitalWrite (ssrPump, 0);
            pumpOn = 0;
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
            digitalWrite (ssrSolenoid, 1);
            solenoidOn = 1;
            digitalWrite (ssrPump, 1);
            pumpOn = 1;
          }
          else {
            brew2ProgPhase = 0;
            digitalWrite (ssrSolenoid, 0);
            solenoidOn = 0;
            digitalWrite (ssrPump, 0);
            pumpOn = 0;
            break;
          }
      }
    }
  }

  //Hot Water
  if ((pumpOn == 0) && (runBrew1 == 0) && (runBrew2 == 0) && (lowWaterSt == 1)) {
    if (switchWaterSt != switchWaterLast) {
      if (switchWaterSt == 1) {
        digitalWrite(ssrPump, 1);
        pumpOn = 1;
      }
    }
  }
  if ((pumpOn == 1) && (runBrew1 == 0) && (runBrew2 == 0) && (brewManual == 0)) {
    if (switchWaterSt != switchWaterLast) {
      if (switchWaterSt == 0) {
        digitalWrite(ssrPump, 0);
        pumpOn = 0;
      }
    }
  }

  //Backflush Program
  if (menu == 5) {
    switch (backflushState) {                                                 //[0,1,2,3] [disabled, reset, run, stop]
      case 1:
        digitalWrite (ssrPump, 0);
        pumpOn = 0;
        digitalWrite (ssrSolenoid, 0);
        solenoidOn = 0;
        backflushCurrentCycle = backflushCycleSet;
        backflushTimer = backflushOnTime;
        backflushPosition = 1;
        backflushState = 0;
        genie.WriteObject (GENIE_OBJ_LED_DIGITS, 0x18, backflushCurrentCycle);
        genie.WriteObject (GENIE_OBJ_LED_DIGITS, 0x19, backflushTimer);
        break;

      case 2:
        if (backflushCurrentCycle > 0) {
          if (backflushPosition == 1) {
            if (backflushTimer > 0) {
              unsigned long currentTimer = millis();
              if (currentTimer - backflushCount >= secondInterval) {
                backflushCount = currentTimer; {
                  backflushTimer--;
                  genie.WriteObject (GENIE_OBJ_LED_DIGITS, 0x19, backflushTimer);
                }
              }
              digitalWrite(ssrSolenoid, 1);
              solenoidOn = 1;
              digitalWrite(ssrPump, 1);
              pumpOn = 1;

            }
            else {
              backflushPosition = 0;
              backflushTimer = backflushDelay;
            }
          }
          if (backflushPosition == 0) {
            if (backflushTimer > 0) {
              unsigned long currentTimer = millis();
              if (currentTimer - backflushCount >= secondInterval) {
                backflushCount = currentTimer; {
                  backflushTimer--;
                  genie.WriteObject (GENIE_OBJ_LED_DIGITS, 0x19, backflushTimer);
                }
              }
              digitalWrite(ssrSolenoid, 0);
              solenoidOn = 0;
              digitalWrite(ssrPump, 0);
              pumpOn = 0;
            }
            else {
              backflushTimer = backflushOnTime;
              backflushPosition = 1;
              backflushCurrentCycle--;
              genie.WriteObject (GENIE_OBJ_LED_DIGITS, 0x18, backflushCurrentCycle);
            }
          }
        }
        else {
          backflushState = 1;
        }
        break;

      case 3:
        digitalWrite (ssrPump, 0);
        pumpOn = 0;
        digitalWrite (ssrSolenoid, 0);
        solenoidOn = 0;
        backflushState = 0;
        backflushPosition = 1;
        break;
    }
  }


  //Run events
  genie.DoEvents();

  //Switch State Monitoring
  switchBrewLast = switchBrewSt;
  switchWaterLast = switchWaterSt;
  switchSteamLast = switchSteamSt;
}


void myGenieEventHandler(void)  {
  genieFrame Event;
  genie.DequeueEvent(&Event);

  //Form Switching
  if (Event.reportObject.cmd == GENIE_REPORT_EVENT) {                                                                     //If the cmd received is from a Reported Event
    if (Event.reportObject.object == GENIE_OBJ_USERBUTTON) {                                                              // If the Reported Message was from a USERBUTTON
      if (Event.reportObject.index == 0) {        // If it was a Brew 1 button
        if (brew1ProgOn == 0) {                                                                                           //Avtivate or deactivate the brew 1 program
          brew1ProgOn = 1;
        }
        else {
          brew1ProgOn = 0;
        }
      }
      if (Event.reportObject.index == 1) {                                                                                // If it was a Brew 2 button
        if (brew2ProgOn == 0) {                                                                                           //Avtivate or deactivate the brew 1 program
          brew2ProgOn = 1;
        }
        else {
          brew2ProgOn = 0;
        }
      }

      //Main Page Setup
      if ((Event.reportObject.index == 4) || (Event.reportObject.index == 6) || (Event.reportObject.index == 11) || (Event.reportObject.index == 15) || (Event.reportObject.index == 17)) {             // If it was a main page button
        genie.WriteObject(GENIE_OBJ_FORM, 0, 0);                                                                                                                     // Change to Form 0, Main page
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x00, brewTimer);
        menu = 0;
      }

      //Brew Program Page Setup
      if ((Event.reportObject.index == 3) || (Event.reportObject.index == 10) || (Event.reportObject.index == 12) || (Event.reportObject.index == 14)) {             // If it was a the Setup or Program buttons
        genie.WriteObject(GENIE_OBJ_FORM, 1, 0);                                                                                                                     // Change to Form 1, Program page
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x01, preinfusionTime1);
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x06, preinfusionDelay1);
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x07, brewProgramTime1);
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x08, preinfusionTime2);
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x09, preinfusionDelay2);
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x0A, brewProgramTime2);
        menu = 1;
      }
      //Temperature Page Setup
      if ((Event.reportObject.index == 5) || (Event.reportObject.index == 7) || (Event.reportObject.index == 13)) {               // If it was a Temp button
        genie.WriteObject(GENIE_OBJ_FORM, 2, 0);                                                                                                                     // Change to Form 2, Temperature page
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x04, brewTempSet);
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x03, brewBuffer);
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x05, steamTempSet);
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x02, steamBuffer);
        menu = 2;
      }
      //Graph Page Setup
      if (Event.reportObject.index == 2) {
        genie.WriteObject(GENIE_OBJ_FORM, 3, 0);
        genie.WriteObject (GENIE_OBJ_LED_DIGITS, 0x0D, temp);
        genie.WriteObject (GENIE_OBJ_LED_DIGITS, 0x1A, Setpoint);
        genie.WriteObject (GENIE_OBJ_LED_DIGITS, 0x0E, pressure);
        menu = 3;
      }
      //PID Page Setup
      if ((Event.reportObject.index == 8) || (Event.reportObject.index == 16) || (Event.reportObject.index == 18)) {
        genie.WriteObject(GENIE_OBJ_FORM, 4, 0);
        genie.WriteObject (GENIE_OBJ_LED_DIGITS, 0x16, consKp * 10);
        genie.WriteObject (GENIE_OBJ_LED_DIGITS, 0x12, consKi * 100);
        genie.WriteObject (GENIE_OBJ_LED_DIGITS, 0x13, consKd * 100);
        genie.WriteObject (GENIE_OBJ_LED_DIGITS, 0x15, aggKp * 10);
        genie.WriteObject (GENIE_OBJ_LED_DIGITS, 0x11, aggKi * 10);
        genie.WriteObject (GENIE_OBJ_LED_DIGITS, 0x14, aggKd * 10);
        genie.WriteObject (GENIE_OBJ_LED_DIGITS, 0x1B, WindowSize);
        menu = 4;
      }
      //Backflush Page Setup
      if ((Event.reportObject.index == 9) || (Event.reportObject.index == 19) || (Event.reportObject.index == 20)) {
        genie.WriteObject(GENIE_OBJ_FORM, 5, 0);
        genie.WriteObject (GENIE_OBJ_LED_DIGITS, 0x0F, backflushCycleSet);              //draw number of programmed cycles
        genie.WriteObject (GENIE_OBJ_LED_DIGITS, 0x10, backflushOnTime);                //draw programmed on time
        genie.WriteObject (GENIE_OBJ_LED_DIGITS, 0x17, backflushDelay);                 //draw programmed delay
        genie.WriteObject (GENIE_OBJ_LED_DIGITS, 0x18, backflushCurrentCycle);
        genie.WriteObject (GENIE_OBJ_LED_DIGITS, 0x19, backflushTimer);
        menu = 5;
      }
    }

    //Setup Buttons
    if (Event.reportObject.object == GENIE_OBJ_4DBUTTON) {                          //If it was a 4d button

      //Brew Program Adjust Menu
      if (Event.reportObject.object == GENIE_OBJ_4DBUTTON) {                          //If it was a 4d button
        //Brew 1
        if (Event.reportObject.index == 8) {                                          // If it was the preinfusion 1 minus button
          preinfusionTime1--;                                                         // Decrement preinfusion
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x01, preinfusionTime1);            // Display new value
          EEPROM.writeInt(preinfusionTime1Mem, preinfusionTime1);
        }
        if (Event.reportObject.index == 9) {                                          // If it was the preinfusion 1 plus button
          preinfusionTime1++;                                                         // inccrement preinfusion
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x01, preinfusionTime1);            // Display new value
          EEPROM.writeInt(preinfusionTime1Mem, preinfusionTime1);
        }
        if (Event.reportObject.index == 10) {                                         // If it was the delay 1 minus button
          preinfusionDelay1--;                                                        // Decrement Delay 1
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x06, preinfusionDelay1);           // Display new value
          EEPROM.writeInt(preinfusionDelay1Mem, preinfusionDelay1);
        }
        if (Event.reportObject.index == 11) {                                         // If it was the delay plus button
          preinfusionDelay1++;                                                        // Increment Delay 1
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x06, preinfusionDelay1);           // Display new value
          EEPROM.writeInt(preinfusionDelay1Mem, preinfusionDelay1);
        }
        if (Event.reportObject.index == 14) {                                         // If it was the preinfusion 1 minus button
          brewProgramTime1--;                                                         // Decrement brew 1 Program Time
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x07, brewProgramTime1);            // Display new value
          EEPROM.writeInt(brewProgramTime1Mem, brewProgramTime1);
        }
        if (Event.reportObject.index == 15) {                                         // If it was the preinfusion plus button
          brewProgramTime1++;                                                         // Increment brew 1 Program Time
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x07, brewProgramTime1);            // Display new value
          EEPROM.writeInt(brewProgramTime1Mem, brewProgramTime1);
        }

        //Brew 2
        if (Event.reportObject.index == 12) {                                         // If it was the preinfusion 2 minus button
          preinfusionTime2--;                                                         // Decrement preinfusion 2
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x08, preinfusionTime2);            // Display new value
          EEPROM.writeInt(preinfusionTime2Mem, preinfusionTime2  );
        }
        if (Event.reportObject.index == 13) {                                         // If it was the preinfusion 2 plus button
          preinfusionTime2++;                                                         // inccrement preinfusion 2
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x08, preinfusionTime2);            // Display new value
          EEPROM.writeInt(preinfusionTime2Mem, preinfusionTime2  );
        }
        if (Event.reportObject.index == 16) {                                         // If it was the delay 2 minus button
          preinfusionDelay2--;                                                        // Decrement Delay 2
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x09, preinfusionDelay2);           // Display new value
          EEPROM.writeInt(preinfusionDelay2Mem , preinfusionDelay2);
        }
        if (Event.reportObject.index == 17) {                                         // If it was the delay 2 plus button
          preinfusionDelay2++;                                                        // Increment Delay 2
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x09, preinfusionDelay2);           // Display new value
          EEPROM.writeInt(preinfusionDelay2Mem , preinfusionDelay2 );
        }
        if (Event.reportObject.index == 18) {                                         // If it was the preinfusion 2 minus button
          brewProgramTime2--;                                                         // Decrement brew 2 Program Time
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x0A, brewProgramTime2);            // Display new value
          EEPROM.writeInt( brewProgramTime2Mem, brewProgramTime2);
        }
        if (Event.reportObject.index == 19) {                                         // If it was the preinfusion plus button
          brewProgramTime2++;                                                         // Increment brew 2 Program Time
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x0A, brewProgramTime2);            // Display new value
          EEPROM.writeInt( brewProgramTime2Mem, brewProgramTime2);
        }
      }
    }
    //Temperature Adjust Menu
    if (menu == 2) {                                                                  // If we are on the temperature page
      //Brew Temperatue
      if (Event.reportObject.object == GENIE_OBJ_4DBUTTON) {                          // If it was a 4d button
        if (Event.reportObject.index == 0) {                                          // If it was the brew temp minus button
          brewTempSet--;                                                              // Decrement brew temp
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x04, brewTempSet);                 // Display new value
          EEPROM.writeDouble(brewTempSetMem , brewTempSet );
        }
        if (Event.reportObject.index == 1) {                                          // If it was the brew temp plus button
          brewTempSet++;                                                              // Increment brew temp
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x04, brewTempSet);                 // Display new value
          EEPROM.writeDouble(brewTempSetMem , brewTempSet );
        }
        if (Event.reportObject.index == 2) {                                          // If it was the brew buffer minus button
          brewBuffer--;                                                               // Decrement brew buffer
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x03, brewBuffer);                  // Display new value
          EEPROM.writeInt(brewBufferMem , brewBuffer );
        }
        if (Event.reportObject.index == 3) {                                          // If it was the brew buffer plus button
          brewBuffer++;                                                               // Increment brew buffer
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x03, brewBuffer);                  // Display new value
          EEPROM.writeInt(brewBufferMem , brewBuffer );
        }

        //Steam Temperature
        if (Event.reportObject.index == 4) {                                          // If it was the steam temp minus button
          steamTempSet--;                                                             // Decrement brew temp
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x05, steamTempSet);                // Display new value
          EEPROM.writeDouble(steamTempSetMem , steamTempSet);
        }
        if (Event.reportObject.index == 5) {                                          // If it was the steam temp plus button
          steamTempSet++;                                                             // Increment brew temp
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x05, steamTempSet);                // Display new value
          EEPROM.writeDouble(steamTempSetMem , steamTempSet );
        }
        if (Event.reportObject.index == 6) {                                          // If it was the steam buffer minus button
          steamBuffer--;                                                              // Decrement steam buffer
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x02, steamBuffer);                 // Display new value
          EEPROM.writeInt(steamBufferMem , steamBuffer );
        }
        if (Event.reportObject.index == 7) {                                          // If it was the steam buffer plus button
          steamBuffer++;                                                              // Increment steam buffer
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x02, steamBuffer);                 // Display new value
          EEPROM.writeInt(steamBufferMem , steamBuffer );
        }
      }
    }
    //PID Adjust Menu
    if (menu == 4) {
      if (Event.reportObject.index == 39) {                                            // If it was the slow Kp minus button
        consKp -= 0.2;                                                                 // Decrement slow Kp value
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x16, consKp * 10);                    // Display new value
        EEPROM.writeDouble(consKpMem, consKp);
      }
      if (Event.reportObject.index == 38) {                                            // If it was the slow Kp plus button
        consKp += 0.2;                                                                 // increment slow Kp value
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x16, consKp * 10);                    // Display new value
        EEPROM.writeDouble(consKpMem, consKp);
      }
      if (Event.reportObject.index == 31) {                                            // If it was the slow Ki minus button
        consKi -= 0.2;                                                                // Decrement slow Ki value
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x12, consKi * 100);                   // Display new value
        EEPROM.writeDouble(consKiMem, consKi);
      }
      if (Event.reportObject.index == 30) {                                            // If it was the slow Ki plus button
        consKi += 0.2;                                                                // increment slow Ki value
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x12, consKi * 100);                   // Display new value
        EEPROM.writeDouble(consKiMem, consKi);
      }
      if (Event.reportObject.index == 32) {                                            // If it was the slow Kd minus button
        consKd -= 0.2;                                                                // Decrement slow Kd value
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x13, consKd * 100);                   // Display new value
        EEPROM.writeDouble(consKdMem, consKd);
      }
      if (Event.reportObject.index == 33) {                                            // If it was the slow Kd plus button
        consKd += 0.2;                                                                // increment slow Kd value
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x13, consKd * 100);                   // Display new value
        EEPROM.writeDouble(consKdMem, consKd);
      }
      if (Event.reportObject.index == 37) {                                            // If it was the fast Kp minus button
        aggKp -= 0.2;                                                                  // Decrement fast Kp value
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x15, aggKp * 10);                     // Display new value
        EEPROM.writeDouble(aggKpMem, aggKp);
      }
      if (Event.reportObject.index == 36) {                                            // If it was the fast Kp plus button
        aggKp += 0.2;                                                                  // increment fast Kp value
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x15, aggKp * 10);                     // Display new value
        EEPROM.writeDouble(aggKpMem, aggKp);
      }
      if (Event.reportObject.index == 29) {                                            // If it was the fast Ki minus button
        aggKi -= 0.2;                                                                  // Decrement fast Ki value
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x11, aggKi * 10);                     // Display new value
        EEPROM.writeDouble(aggKiMem, aggKi);
      }
      if (Event.reportObject.index == 28) {                                            // If it was the fast Ki plus button
        aggKi += 0.2;                                                                  // increment fast Ki value
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x11, aggKi * 10);                     // Display new value
        EEPROM.writeDouble(aggKiMem, aggKi);
      }
      if (Event.reportObject.index == 34) {                                            // If it was the fast Kd minus button
        aggKd -= 0.2;                                                                  // Decrement fast Kd value
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x14, aggKd * 10);                     // Display new value
        EEPROM.writeDouble(aggKdMem, aggKd);
      }
      if (Event.reportObject.index == 35) {                                            // If it was the fast Kd plus button
        aggKd += 0.2;                                                                  // increment fast Kd value
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x14, aggKd * 10);                     // Display new value
        EEPROM.writeDouble(aggKdMem, aggKd);
      }

      if (Event.reportObject.index == 41) {                                            // If it was the windowsize minus button
        WindowSize -= 10;                                                              // Decrement windowsize
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x1B, WindowSize);                     // Display new value
        EEPROM.writeLong(WindowSizeMem, WindowSize);
        tempPID.SetOutputLimits(0, WindowSize);
      }
      if (Event.reportObject.index == 42) {                                            // If it was the windowsize plus button
        WindowSize += 10;                                                              // increment windowsize
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x1B, WindowSize);                     // Display new value
        EEPROM.writeLong(WindowSizeMem, WindowSize);
        tempPID.SetOutputLimits(0, WindowSize);
      }
    }
    //Backflush Menu
    if (menu == 5) {
      if (Event.reportObject.index == 20) {                                            // If it was the program cycles minus button
        backflushCycleSet--;                                                           // Decrement CycleSet
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x0F, backflushCycleSet);              // Display new value
        EEPROM.write(backflushCycleMem, backflushCycleSet);
      }
      if (Event.reportObject.index == 21) {                                            // If it was the program cycles plus button
        backflushCycleSet++;                                                           // Increment CycleSet
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x0F, backflushCycleSet);              // Display new value
        EEPROM.write(backflushCycleMem, backflushCycleSet);
      }
      if (Event.reportObject.index == 22) {                                            // If it was the program ontime minus button
        backflushOnTime--;                                                             // Decrement on time
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x10, backflushOnTime);              // Display new value
        EEPROM.write(backflushOnTimeMem, backflushOnTime);
      }
      if (Event.reportObject.index == 23) {                                            // If it was the program ontime plus button
        backflushOnTime++;                                                             // Increment on time
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x10, backflushOnTime);              // Display new value
        EEPROM.write(backflushOnTimeMem, backflushOnTime);
      }
      if (Event.reportObject.index == 24) {                                            // If it was the program delay time minus button
        backflushDelay--;                                                              // Decrement delay time
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x17, backflushDelay);              // Display new value
        EEPROM.write(backflushOnTimeMem, backflushDelay);
      }
      if (Event.reportObject.index == 25) {                                            // If it was the program delay time plus button
        backflushDelay++;                                                              // Increment delay time
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x17, backflushDelay);              // Display new value
        EEPROM.write(backflushOnTimeMem, backflushDelay);
      }
      if (Event.reportObject.index == 26) {                                            // If it was the backflush reset button
        backflushState = 1;
      }
      if (Event.reportObject.index == 27) {                                            // If it was the backflush run button
        backflushState = 2;
      }
      if (Event.reportObject.index == 40) {                                            // If it was the backflush stop button
        backflushState = 3;
      }
    }
  }
}
