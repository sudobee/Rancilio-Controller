//Rancilio Coffee Machine Controller

//Pin Names
#define boiler_pin 1                        //Define SSR boiler relay output
#define main_power_pin  2                   //Define Main Power Switch
#define brew_pin 3                          //Define Brew Switch
#define steam_pin 4                         //Define Steam Switch
#define temp_pin A0                         //Define Temp input
#define up_pin A1                           //Define Pushbutton 1 Up
#define down_pin A2                         //Define Pushbutton 2 Down
#define left_pin A3                         //Define Pushbutton 3 Left
#define right_pin A2                        //Define Pushbutton 4 Right
#define OK_pin A3                           //Define Pushbutton 5 OK, not used

//Constants and Variables
unsigned long previousMillis = 0;
const long interval = 1800;                 //Serial debug print delay

unsigned long lcdcount;
const long lcdint = 100;                    //lcd refresh rate

unsigned long buttoncount;
const long buttonint = 50;                  //lcd refresh rate

unsigned long secondcount;
const long secondint = 1000;                //second counter

int menu = 1;                               //Init and default menu
int templen = 0;                            //Init temp length var

int upstate = 0;                            //button state variables
int lastupstate = 0;
int downstate = 0;
int lastdownstate = 0;
int leftstate = 0;
int lastleftstate = 0;
int rightstate = 0;
int lastrightstate = 0;

int brewswitch = 0;
int brewswitchlast = 0;
int brewtime = 0;

//Initialise
#include <LiquidCrystal.h>
LiquidCrystal lcd (12, 11, 7, 8, 9, 10);


void setup() {

  //Init
  Serial.begin(9600);                       //Start serial
  lcd.begin(16, 2);                         //Start LCD

  //Pin Setup
  pinMode(boiler_pin, OUTPUT);              // set SSR relay output
  pinMode(main_power_pin, INPUT);           // set main switch input
  pinMode(brew_pin, INPUT);                 // set brew switch input
  pinMode(steam_pin, INPUT);                // set steam switch input
  pinMode(temp_pin, INPUT);                 // set temp input
  pinMode(up_pin, INPUT);                   //control input
  pinMode(down_pin, INPUT);                 //control input
  pinMode(left_pin, INPUT);                 //control input
  pinMode(right_pin, INPUT);                //control input
  pinMode(OK_pin, INPUT);                   //control input
  pinMode(13, OUTPUT);                      //set on board LED output
}

void loop() {

  //Variables
  int temp = (analogRead(temp_pin) / 6);            //Get Temperature
  int main_power = digitalRead(main_power_pin);     //Get Power Switch
  int brew = digitalRead(brew_pin);                 //Get Brew Switch
  int steam = digitalRead(steam_pin);               //Get Steam Switch

  int brewtemp = 105;                               //Define and set brewtemp value - add menu adjust later
  int brewbuffer = 2;                               //Allows x degrees swing before switching boiler
  int brewhigh = (brewtemp + brewbuffer);
  int brewlow = (brewtemp - brewbuffer);

  int steamtemp = 145;                              //Define and set steamtemp value - add menu adjust later
  int steambuffer = 2;                              //Allows x degrees swing before switching boiler
  int steamhigh = (steamtemp + steambuffer);
  int steamlow = (steamtemp - steambuffer);

  boolean boileron;                                 //Boiler power status

  //Control Loop

  if (main_power == HIGH)                                   //Test power switch
  {
    if (steam == HIGH) {                                    //Test Steam Switch
      if ((temp > 5) && (temp <= steamlow)) {               //Test if temperature is under steam temp
        digitalWrite(boiler_pin, HIGH);                     //Turn on boiler
        boileron = true;                                    //Track boiler status
      }
      if (temp >= steamhigh) {                              //Steam temperature reached
        digitalWrite(boiler_pin, LOW);                      //Turn Off boiler
        boileron = false;                                   //Track boiler status
      }
    }
    else {                                                  //Steam Switch is off so run boiler to brew temp
      if ((temp > 5) && (temp <= brewlow)) {                //Test if temperature is under brew temp
        digitalWrite(boiler_pin, HIGH);                     //Turn on boiler
        boileron = true;                                    //Track boiler status
      }
      if (temp >= brewhigh) {                               //Brew temperature reached
        digitalWrite(boiler_pin, LOW);                      //Turn Off boiler
        boileron = false;                                   //Track boiler status
      }
    }
  }
  else {                                                    //Power switch is off
    digitalWrite(boiler_pin, LOW);                          //Turn Off boiler
    boileron = false;                                       //Track boiler status
  }

  if (boileron = true) {                                    //On board LED for boiler tracking
    digitalWrite(13, HIGH);
  }
  else {
    digitalWrite(13, LOW);
  }

  //Front End Init.
  int menulast = menu;                                       //for menu/display output cleairng
  int templenlast = templen;                                 //adjusts temperature print out

  //Menus
  unsigned long currentlcdcount = millis();                  //LCD print delay to reduce flicker
  if (currentlcdcount - lcdcount >= lcdint) {
    lcdcount = currentlcdcount; {

      switch (menu) {
        case 1:                                     //main temperature display
          if (temp <= 99) {
            templen = 2;
            lcd.setCursor(3, 1);
            lcd.print ("C");
          }
          if (temp > 99) {
            templen = 3;
            lcd.setCursor(4, 1);
            lcd.print ("C");
          }
          if (templenlast != templen) {
            lcd.clear();
          }
          lcd.setCursor(0, 0);
          lcd.print("Temperature:");
          lcd.setCursor(0, 1);
          lcd.print(temp);
          break;
        case 2:                                     //Set brew temp
          lcd.setCursor(0, 0);
          lcd.print("Set Brew Temperature");
          lcd.setCursor(0, 1);
          lcd.print(brewtemp);
          break;
        case 3:
          lcd.setCursor(0, 0);                      //Set brew buffer
          lcd.print("Set Brew Buffer");
          lcd.setCursor(0, 1);
          lcd.print(brewbuffer);
          break;
        case 4:
          lcd.setCursor(0, 0);                      //Set steam temp
          lcd.print("Set Steam Temperature");
          lcd.setCursor(0, 1);
          lcd.print(steamtemp);
          break;
        case 5:
          lcd.setCursor(0, 0);                      //Set steam buffer
          lcd.print("Set Steam Buffer");
          lcd.setCursor(0, 1);
          lcd.print(steambuffer);
          break;
        default:                                    //shit be broke
          lcd.setCursor(0, 0);
          lcd.print("SadFaces");
          lcd.setCursor(0, 1);
          lcd.print(":( :( :( :( :(");
          break;
      }

      //Brew Timer
      brewswitch = digitalRead(brew_pin);
      if (brewswitch != brewswitchlast) {
        if (brewswitch == HIGH) {                                         //Brew switch rising edge
          menu = 1;                                                       //go to temp page
          goto brewtimer;                                                 //start brew timer
        }
        else {                                                            //Brew switch falling edge
          brewtime = 0;                                                   //reset brew timer
          lcd.clear();                                                    //hide brew timer
        }
      }
brewtimer:                                                                //this didnt want to work in the if statements
      if (brewswitch == HIGH) {                                           //recheck state
        unsigned long currentsecondcount = millis();                      //1 second counter
        if (currentsecondcount - secondcount >= secondint) {
          secondcount = currentsecondcount; {
            brewtime = (brewtime + 1);                                    //1 second intervals, starts at 1
            lcd.setCursor(14, 1);
            lcd.print(brewtime);
          }
        }
      }
      brewswitchlast = brewswitch;                                         //track brew switch for rising/falling edge
      //Menu clear
      if (menulast != menu) {                                              //clear unused characters on menu swap
        lcd.clear();
      }
    }
  }


  //Menu Control
  unsigned long currentbuttoncount = millis();
  if (currentbuttoncount - buttoncount >= buttonint) {                     //button input lag to reduce bounce
    buttoncount = currentbuttoncount; {

      upstate = digitalRead(up_pin);                                       //Up pin variable adjust
      if (upstate != lastupstate) {
        if (upstate == HIGH) {
          if (menu == 2) {
            brewtemp = (brewtemp + 1);
          }
          if (menu == 3) {
            brewbuffer = (brewbuffer + 1);
          }
          if (menu == 4) {
            steamtemp = (steamtemp + 1);
          }
          if (menu == 5) {
            steambuffer = (steambuffer + 1);
          }
        }
      }
      downstate = digitalRead(down_pin);                                   //down pin variable adjust
      if (downstate != lastdownstate) {
        if (downstate == HIGH) {
          if (menu == 2) {
            brewtemp = (brewtemp - 1);
          }
          if (menu == 3) {
            brewbuffer = (brewbuffer - 1);
          }
          if (menu == 4) {
            steamtemp = (steamtemp - 1);
          }
          if (menu == 5) {
            steambuffer = (steambuffer - 1);
          }
        }
      }
      leftstate = digitalRead(left_pin);                                   //left pin menu/display scrolling
      if (leftstate != lastleftstate) {
        if (leftstate == HIGH) {
          menu = (menu - 1);
        }
      } 
      rightstate = digitalRead(right_pin);                                 //right pin menu/display scolling
      if (rightstate != lastrightstate) { 
        if (rightstate == HIGH) {
          menu = (menu + 1);
        }
      }
    }
    lastupstate = upstate;                                                 //button state tracking for one menu click per press
    lastdownstate = downstate;
    lastleftstate = leftstate;
    lastrightstate = rightstate;
  }

  /*
    //Serial debug
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;

      {
        Serial.print("brewtime  ");
        Serial.println(brewtime);
        Serial.print("Brewswitch:  ");
        Serial.println(brewswitch);
        Serial.print("Button 3:  ");
        Serial.println();
        Serial.print("Menu:  ");
        Serial.println(menu);
        Serial.println();
        Serial.println();
        Serial.print("Temp:  ");
        Serial.println(temp);
        Serial.println();
        Serial.print("Main Power:  ");
        Serial.println(main_power);
        Serial.print("Steam:  ");
        Serial.println(steam);
        Serial.print("Boiler:  ");
        Serial.println(boileron);
        Serial.println();
        Serial.println();
      }
    }              */
}

