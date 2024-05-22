// STRIPPED FILE ]
#define default_val \
  { 0x0, 0xE6, 0x0, 0xC2, 0x1, 0x1C, 0x2, 0x38, 0x4 }
//
#define abs(x) ((x) > 0 ? (x) : -(x))

// #define __SERIAL_BEGIN_
// #define __DEBUG_LCD_DEMO_
// #define __INTIAL_TIME_ESTIMATION_
// #define __LOOP_TIME_ESTIMATION_
// #define _SoundALarm_ON_
// #define __Servo_Debug_

#define __pH_Meter__PRESENT_
#define cal_val 9.7
// #define __Update_Clock_

#include <digitalWriteFast.h>


#include <LiquidCrystal.h>
const uint8_t rs = 27, en = 26, d4 = 25, d5 = 24, d6 = 23, d7 = 22;
const uint8_t col = 20, row = 4;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);  // object creation
#define contrastPin 11                      //pwm capable pin //not necessary for 1604 display must for 2004 display
#define backlightPin 12                     //pwm capable pin
uint8_t contrast_val = 200;                 //does not work in this lcd
uint8_t backlight_val = 50;                 //can be used to dim the backlight as per light measurement backlight = (currentLight/5) + 15

enum lcd_option { DoNothing,
                  LCD_Demo,
                  setupEEPROM,
                  Update_EEPROM,
                  LCD_Display };
void lcdmenu(lcd_option option);                 //declaration of lcdmenu()
void lcdLnClr(uint8_t lineNo, uint8_t pos = 0);  //declaration of lcdLnClr() small function to clear line and set postition of cursor to pos handles funny LCDd
void lcdPos(uint8_t lineNo, uint8_t pos = 0);


#define Rot_Encoder_CLK 28
#define Rot_Encoder_DT 29
#define Rot_Encoder_SW 30  //pressed when low
#define Mode_Button 31     //use internal pull up register for const when pressed low
#define Feed_Button 2      //it works here? INPUT_PULLUP used  active when low

enum Rot_Encoder_Event { NO_CHANGE,
                         CLOCKWISE,
                         ANTICLOCKWISE,
                         SW_BUTTON };

uint16_t getLDR();
uint8_t getPH();
Rot_Encoder_Event Rot_Encoder_Poll();
bool MB_Event();
void alarm(bool state);
void Blink(uint16_t blinkDuration, uint8_t intensity);
void retriveFromEEPROM();
void saveToEEPROM();
void ReadCurrents();
void ControlState();
void setup();
void loop();
void lcdmenu(lcd_option option);
void f_setTemp();  // need to save
void f_setLux();
void f_setTime(byte sel);
void inputParameters();

#define ONE_WIRE_BUS 10   //pin to read for temp sensor  digital one wire pin
#define currentPinLux A8  //use internal pullup resistor pin to read form ldr  valid
#define currentPin_pH A9  //pin to read from pH meter // analog valid

#include <RTClib.h>
RTC_DS3231 rtc;  //uses tx rx  I2C channel autosearch mode pin used 14 15
DateTime rtc_now;
int16_t checkFeedTime = 0;
byte time_hh;
byte time_min;

#include <OneWire.h>
#include <DallasTemperature.h>
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature oneWireSensors(&oneWire);
DeviceAddress insideThermometer;  // array to hold address inside thermometer
uint16_t getCelsius(DeviceAddress deviceAddress);

#include <stdint.h>
#include <EEPROM.h>
#include <Servo.h>

uint16_t getCelsius(DeviceAddress deviceAddress) {  //working  dont touch

  oneWireSensors.requestTemperatures();

  uint16_t tempC = 10 * oneWireSensors.getTempCByIndex(0);  //first tempSensor
  return tempC;
}

uint16_t getLDR() {  // returns Lux normal scale
  /*
  Condition	      Typical Lux                 analog reading of current LDR
  Sunlight	    	107527                    1022
  Full Daylight	 	10752                     1022
  Overcast Day	 	1075                      1022
  mobile flash light 25 cm away             690
  ----------> decision to turn light       ~600
  15 W LED bulb overhead in 10 foor room    425
  Very Dark Day		107
  Twilight	    	10.8                      <40
  Deep Twilight		1.08
  Full Moon	    	0.108
  Quarter Moon  	0.0108
  Starlight	    	0.0011
  Overcast Night	0.0001
  */

  return analogRead(currentPinLux);
}  //getLDR()

uint8_t getPH() {  //returns in pH*10    //returns deci scale  //running average  //non blocking and avioids sudden jumps
  #ifdef __pH_Meter__PRESENT_
    const uint8_t size = 10;
    static float pHs[size], sum = 0;  //static are by default initialized 0
    static uint8_t i = 0;

    float val = (21.34 + cal_val) - 5.7 * analogRead(currentPin_pH) * 5 / 1024;
    if (i < size) {
      pHs[i] = val;
      i++;
      sum += val;
      return sum * 10 / i;
    } else {  //shift
      sum -= pHs[0];
      for (int j = 0; j < size - 1; j++) {
        pHs[j] = pHs[j + 1];  //pHs[last-1] is freed
      }
      pHs[size - 1] = val;
      sum += val;
    }
    return sum;  //in deci. scale
  #else
    return 70;
  #endif
}  //getPH()

#define servoMinDegrees 80  //the limits to servo movement
#define servoMaxDegrees 130
#define servoRunTime 3
uint8_t servoRuns = 0;

Servo servoFeed;  //create servo object to control a servo

struct {
  bool FoodServe = LOW;
  bool FoodServe1 = LOW;  //reset these at  control at hhmm 0000 or hhmm=0;
  bool FoodServe2 = LOW;
  bool FilterCtrlPin = LOW;  // normal state is low --> filter pump on
  bool SW_Current = HIGH;
  bool SW_Last = HIGH;
  bool MB_Current = HIGH;
  bool MB_Last = HIGH;        //is high when button is not pressed
  bool RE_CLK_Current = LOW;  //rotary encoder
  bool RE_CLK_Last = LOW;
} state;

union {
  struct {
    byte flag;                     //1 bool      valid data means flag 1
    uint16_t setTemp;              //2 byte      in temp*10 scale
    uint16_t setLux;               //2 byte      int lux*10 scale
    uint16_t feedTime1;            //2 byte
    uint16_t feedTime2;            //2 byte      in minutes  hh*60+mm            //overwritten by one eeprom
  };                               //9 bytes
  byte byteData[9] = default_val;  //default values
} x;

uint8_t range_pH = 15;
uint8_t normal_pH_7 = 70;  //used deci
uint8_t current_pH;        // use deci range 0 to 140 val*10
uint16_t currentTemp;      // use deci for both in val*10 scale  Range 10 unit
uint16_t currentLight;     // uint16_t to accomodate 1024 levels the output of ADC  2 bytes

unsigned long millisCurrent = 0;  //stores the value of millis() in each iteration of loop()

  #ifdef __LOOP_TIME_ESTIMATION_
unsigned long millisEndofLoop = 0;
  #endif

uint16_t interval = 500;  //number of millisecs used in read lcd and blinks

byte Char_centigrade[8] = {
  B01000,
  B10100,
  B01000,
  B00011,
  B00100,
  B00100,
  B00011,
  B00000
};
byte Char_Lux[8] = {
  B10101,
  B01110,
  B01110,
  B11011,
  B01110,
  B01110,
  B10101,
  B00000
};
byte Char_PH[8] = {
  B00101,
  B00101,
  B00111,
  B00101,
  B11000,
  B10100,
  B11000,
  B10000
};
byte Char_F1[8] = {
  B00010,
  B00010,
  B00010,
  B00000,
  B11100,
  B10000,
  B11000,
  B10000
};
byte Char_F2[8] = {
  B00110,
  B00001,
  B00010,
  B00111,
  B11100,
  B10000,
  B11000,
  B10000
};
byte Char_ArrowRight[8] = {
  B00000,
  B00100,
  B00010,
  B11111,
  B00010,
  B00100,
  B00000,
  B00000
};
byte Char_ArrowLeft[8] = {
  B00000,
  B00100,
  B01000,
  B11111,
  B01000,
  B00100,
  B00000,
  B00000
};
byte Char_ArrowUp[8] = {
  B00100,
  B01110,
  B11111,
  B00100,
  B00100,
  B00100,
  B00100,
  B00000
};
byte Char_ArrowDown[8] = {
  B00000,
  B00100,
  B00100,
  B00100,
  B00100,
  B11111,
  B01110,
  B00100
};
byte Char_AA[2][8] = {
  { B00010,
    B00101,
    B00111,
    B00101,
    B01000,
    B10100,
    B11100,
    B10100 },
  { B01000,
    B10100,
    B11100,
    B10100,
    B00010,
    B00101,
    B00111,
    B00101 }
};

Rot_Encoder_Event Rot_Encoder_Poll() {

  unsigned long millisTimeout = 4000;
  unsigned long millisInitial = millis();
  while ((millis() - millisInitial) < millisTimeout) {

    state.RE_CLK_Current = digitalReadFast(Rot_Encoder_CLK);
    if (state.RE_CLK_Current != state.RE_CLK_Last) {  // edge
      state.RE_CLK_Last = state.RE_CLK_Current;

      if (!state.RE_CLK_Current) {              //falling edge
        if (digitalReadFast(Rot_Encoder_DT)) {  // if high rotating anticlockwise

          return ANTICLOCKWISE;

        } else {

          return CLOCKWISE;
        }
      }
    }

    state.SW_Current = digitalReadFast(Rot_Encoder_SW);
    if (state.SW_Last != state.SW_Current) {  // detecting falling edge
      state.SW_Last = state.SW_Current;
      if (!state.SW_Current) {

        return SW_BUTTON;
      }
    }
  }

  return NO_CHANGE;

}  // Rot_Encoder_Poll()

bool MB_Event() {                               //Mode Button Event
  state.MB_Current = digitalRead(Mode_Button);  //normal is the high
  if (state.MB_Last != state.MB_Current) {      // detecting falling edge
    state.MB_Last = state.MB_Current;
    if (!state.MB_Current) {  //if state.MB_Currnt = 0

      return 1;
    }
  }
  state.MB_Last = state.MB_Current;
  return 0;
}  //MB_Event()

#define controlPinUnknown 32   // notworking cut off just wire available outside
#define controlPinLighting 33  //for controlling aquarium light normal is low.
#define controlPinHeater 34    // heater default state low  while high cutoff the heater
#define controlPinFilter 35    // default state low high cutoff
#define controlPinAlarm 36     //for the alarm indicatro led and or buzzer
#define controlPinAlarmLED 37
#define controlPinFoodServo 8  //pwn  needed servo motor turn pin. Normal low. introduce feed when high

#define onBoardLedPin 13  //onBoard LED used to indicate operation state by blinking every x loops

void alarm(bool state) {  //works properly

  digitalWriteFast(controlPinAlarmLED, state);

  #ifdef _SoundALarm_ON_
  digitalWriteFast(controlPinAlarm, state);
  #endif

}  //alarm(bool state)

void Blink(uint16_t blinkDuration, uint8_t intensity = 255) {  //default is half power led intensity offa t below 150
  static bool state;
  static unsigned long millisLastBoardLed;
  if (millisCurrent - millisLastBoardLed >= blinkDuration) {
    if (state == 1) {
      analogWrite(onBoardLedPin, intensity);  //real Toggle herein
    } else {
      digitalWrite(onBoardLedPin, LOW);
    }
    state = !state;
    millisLastBoardLed += blinkDuration;  //increment the millis
  }
}  //Blink()

void retriveFromEEPROM() {

  if (EEPROM.read(0) == 1) {  //first address is 0   //valid data is from byte 1
    for (uint8_t i = 1; i < sizeof(x); i++) {
      x.byteData[i] = EEPROM.read(i);
    }
  } else {
    lcdmenu(setupEEPROM);
  }

}  //retriveFromEEPROM()

void saveToEEPROM() {
  x.flag = 1;
  for (uint8_t i = 0; i < sizeof(x); i++) {
    EEPROM.update(i, x.byteData[i]);
  }
}  //saveToEEPROM()

// void ReadCurrents() {                   //need updating
//   static unsigned long millisLastRead=0;  //will store last time anlog input were read
//   if (millisCurrent - millisLastRead >= interval) {
//     currentTemp = getCelsius(insideThermometer);  //deci scale
//     currentLight = getLDR();                      //ok  // normal scale
//     current_pH = getPH();                         //ok  set ph into deci
//     millisLastRead = millisCurrent;               //putting most recent value of millis in last read one.
//   }
// }  //ReadCurrents()

void ControlState() {  //need debug here check the outputs  //nonblocking
  bool flag_alarm = 0;
  static uint8_t servoPosition = servoMinDegrees;
  static bool servoDirection = HIGH;
  static uint16_t filterOffEvent;

  if (currentTemp < x.setTemp - 5) {          //turn on at .5 degree lower than set
    digitalWriteFast(controlPinHeater, LOW);  //becomes LOW  heater on
  }

  if (currentTemp >= x.setTemp) {              //hysterisis loop included to stop frequent on and off .
    digitalWriteFast(controlPinHeater, HIGH);  //becomes HIGH   heater off
  }

  if (currentLight < x.setLux) {                 //introducing hysteresis to prevent flickering
    digitalWriteFast(controlPinLighting, HIGH);  //becomes high Light on
    //digitalWriteFast(onBoardLedPin, LOW);
    Blink(interval, 10);
  }

  if (currentLight > x.setLux + 100) {
    Blink(interval);
    digitalWriteFast(controlPinLighting, LOW);  //becomes low lights off
  }

  if (abs(current_pH - normal_pH_7) > range_pH) {  //evaluated value can be negetive

    flag_alarm = 1;
  }

  if (x.setTemp > currentTemp + 20) {  //stored units are decis ie 20.0 stored as 200 since it is unsigned the diff is always positi

    flag_alarm = 1;
  }

  if (currentTemp > x.setTemp + 20) {  //stored units are decis ie 20.0 stored as 200

    flag_alarm = 1;
  }

  if (flag_alarm) {
    alarm(HIGH);
  } else {
    alarm(LOW);
  }

  if (checkFeedTime == x.feedTime1) {
    if (state.FoodServe1 == LOW) {  //pin low
      state.FoodServe = HIGH;       //becomes high servo on
      state.FoodServe1 = HIGH;
    }

  } else {
    state.FoodServe1 = LOW;
  }

  if (checkFeedTime == x.feedTime2) {
    if (state.FoodServe2 == LOW) {
      state.FoodServe = HIGH;  //pin high food served
      state.FoodServe2 = HIGH;
    }

  } else {
    state.FoodServe2 = LOW;
  }

  if (digitalRead(Feed_Button) == LOW) {  //active low

    state.FoodServe = HIGH;
  }

  if (state.FoodServe == HIGH) {  //wrapping it in funtion does not work why??
    state.FilterCtrlPin = HIGH;   //means filter off
    //digitalWriteFast(controlPinFilter, state.FilterCtrlPin);
    filterOffEvent = checkFeedTime;

    servoFeed.attach(controlPinFoodServo);

    if (servoRuns < servoRunTime) {
      // indicate optimized version of above

      if (servoPosition >= servoMinDegrees) {  //blocking taking 800ms  how to make it unblocking??
      #ifdef __Servo_Debug_
            Serial.print("before ");
            Serial.println(servoPosition);
      #endif

        if (servoDirection) {
          servoPosition++;
          //servoDirection=servoMaxDegrees;
      #ifdef __Servo_Debug_
              Serial.print("+");
              Serial.println(servoPosition);
              Serial.print("read ");
              Serial.println(servoFeed.read());
      #endif
        } else {
          servoPosition--;
          //servoPosition=servoMinDegrees;    //this does not work funnny why not sure?
      #ifdef __Servo_Debug_

              Serial.print("-");
              Serial.println(servoPosition);
      #endif
        }
        if (servoPosition >= servoMaxDegrees) {
          servoDirection = LOW;
      #ifdef __Servo_Debug_

              Serial.print("servoDirection");
              Serial.println(servoDirection);
      #endif
        }
        if (servoPosition <= servoMinDegrees) {
          servoDirection = HIGH;
      #ifdef __Servo_Debug_

              Serial.print("servoDirection");
              Serial.println(servoDirection);
      #endif
          servoRuns++;
        }

       //while (servoPosition != servoFeed.read()) {  while loop not needed any way its not working as planned only increment is working but decrement is not may be != servoFeed.read() checks floor val
    #ifdef __Servo_Debug_

            Serial.print("read servoPosition");
            Serial.println(servoPosition);
    #endif
          servoFeed.write(servoPosition);

          delay(15);
        //}
      }
    } else {
      state.FoodServe = LOW;
      //filterOffEvent = checkFeedTime;
      servoFeed.detach();
      servoRuns = 0;
    }
  }

  if (state.FilterCtrlPin) {  //ie filter is off						// if filter pin is high toggle it on after delay of 30 min
    if (checkFeedTime - filterOffEvent >= 30) {
      state.FilterCtrlPin = LOW;  // low means filter pump on countdown to start filter pump started
      //digitalWriteFast(controlPinFilter, state.FilterCtrlPin);
    }
  }
  digitalWriteFast(controlPinFilter, state.FilterCtrlPin);

}  //ControlState()

void setup() {
  millisCurrent = millis();
  #if defined __SERIAL_BEGIN_ || defined __INTIAL_TIME_ESTIMATION_ || defined __LOOP_TIME_ESTIMATION_
    Serial.begin(57600);
    Serial.println(F("Starting SeveralThingsAtTheSameTimeAquarium"));
  #endif

  #ifdef __INTIAL_TIME_ESTIMATION_
    Serial.print(millisCurrent);
    Serial.print(F(" ms \nPassed Macro Values\n"));
    Serial.println(__DATE__);
    Serial.println(__TIME__);
    Serial.print(F("current millis"));
    Serial.print(millis());
    Serial.println(" ms");
  #endif

  if (!rtc.begin()) {
    #ifdef __SERIAL.BEGIN__
      Serial.println(F("Couldn't find RTC"));
      Serial.flush();
    #endif
    delay(100);
  }

  if (rtc.lostPower()) {
  #ifdef __SERIAL.BEGIN__
    Serial.println(F("RTC lost power, let's set the time!"));
  #endif
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));  //adjust here for the initial time
  }

  #ifdef __Update_Clock_
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));  //adjust here for the initial time
    rtc_now = rtc.now();
    rtc.adjust(DateTime(rtc_now.year(), rtc_now.month(), rtc_now.day(), rtc_now.hour(), rtc_now.minute(), rtc_now.second() + 4));  //adjusting time working

    Serial.begin(57600);

    Serial.print(millisCurrent);
    Serial.print(F(" ms \nPassed Macro Values\n"));
    Serial.println(__DATE__);
    Serial.println(__TIME__);
    Serial.print(F("current millis :"));
    Serial.print(millis());
    Serial.println(" ms");
    Serial.print(F("Clock Synced Upload with  \n do  #define__Update_Clock_    Commented Out"));

    lcd.begin(col, row);
    lcd.clear();
    lcdLnClr(0);
    lcd.print(F("Clock Synced"));
    lcdLnClr(1);
    lcd.print(F("Upload with "));
    lcdLnClr(2);
    lcd.print(F("__Update_Clock_ "));
    lcdLnClr(3);
    lcd.print(F("Commented Out"));
  #endif


  #ifndef __Update_Clock_
    lcd.begin(col, row);
    lcd.clear();
    lcdLnClr(0);
    lcd.print(F("Aquarium"));
    lcdLnClr(1, 4);
    lcd.print(F("Automation"));

    servoFeed.attach(controlPinFoodServo);  //needed due to funny servo jitter while powering on which shifts the position to about 5  degree
    servoFeed.write(servoMinDegrees);

    oneWireSensors.begin();

    #ifdef __SERIAL.BEGIN__
      Serial.println(F("Starting SeveralThingsAtTheSameTimeAquarium"));
    #endif

    pinMode(contrastPin, OUTPUT);  //no effect for 1604
    pinMode(backlightPin, OUTPUT);
    analogWrite(contrastPin, contrast_val);    //set some contrast
    analogWrite(backlightPin, backlight_val);  //set backlight on

    pinModeFast(Rot_Encoder_CLK, INPUT);
    pinModeFast(Rot_Encoder_DT, INPUT);
    pinModeFast(Rot_Encoder_SW, INPUT_PULLUP);
    pinModeFast(Mode_Button, INPUT_PULLUP);  //low when pressed
    pinModeFast(Feed_Button, INPUT_PULLUP);  //low when pressed

    pinModeFast(currentPinLux, INPUT);
    pinModeFast(currentPin_pH, INPUT);

    pinModeFast(controlPinHeater, OUTPUT);
    pinModeFast(controlPinFilter, OUTPUT);
    pinModeFast(controlPinLighting, OUTPUT);
    pinModeFast(controlPinAlarm, OUTPUT);
    pinModeFast(controlPinAlarmLED, OUTPUT);
    pinModeFast(onBoardLedPin, OUTPUT);
    
    //no need for these here as it will be contolled in few ms by control()  
    //digitalWriteFast(controlPinHeater, LOW);  //initial state of Heater pin heater is on
    //digitalWriteFast(controlPinFilter, LOW);  //initial state of Filter pin Filter is on
    //digitalWriteFast(controlPinLighting, LOW);
    //digitalWriteFast(controlPinAlarm, LOW);
    //digitalWriteFast(controlPinAlarmLED, LOW);
    //digitalWriteFast(onBoardLedPin, LOW);  //initial state of LED pin  LED off

    delay(250);
    servoFeed.detach();  //this gives sufficient time to move to initial poweron moves about 5 degree to anticlockwise during each poweron cycle

    retriveFromEEPROM();
      #ifdef __DEBUG_LCD_DEMO_
        lcdmenu(LCD_Demo);
      #endif
  #endif

}  //setup()

#ifdef __Update_Clock_    //loop()
  void loop() {
  }  //loop()
#endif

#ifndef __Update_Clock_   //loop()
  void loop() {

    millisCurrent = millis();  //note internal clock in millisecond

    #ifdef __LOOP_TIME_ESTIMATION_
      if (millisCurrent - millisEndofLoop > 100) {
        Serial.print(millisCurrent - millisEndofLoop);
        Serial.println(F("ms IN THE LOOP"));
      }
    #endif

    rtc_now = rtc.now();
    time_hh = rtc_now.hour();
    time_min = rtc_now.minute();

    checkFeedTime = time_hh * 60 + time_min;

    //ReadCurrents();

    lcdmenu(LCD_Display);

    if (MB_Event()) {
      lcdmenu(Update_EEPROM);  //loop stopped while reading ie set button is kept push
    }

    ControlState();

    #ifdef __LOOP_TIME_ESTIMATION_
    millisEndofLoop = millisCurrent;
    #endif

  }  //loop()
#endif

void lcdPos(uint8_t lineNo, uint8_t pos = 0) {  //nonblocking
  if (lineNo >= 2) {
    pos -= 4;
  }
  lcd.setCursor(pos, lineNo);
}  //lcdPos

void lcdLnClr(uint8_t lineNo, uint8_t pos = 0) {  //nonblocking
  lcdPos(lineNo);
  lcd.print(F("                "));
  lcdPos(lineNo, pos);  //cause i got the funny LCD
}  //lcdLnClr

void lcdTimePrint(uint8_t val) {
  if (val < 10) {
    lcd.print(0);
  }
  lcd.print(val);
}

void lcdmenu(lcd_option option) {  //stable works do not tweak further  nonblocking except when entering setup

  //time_hh = rtc_now.hour();       //variable is set in the loop
  //time_min = rtc_now.minute();
  uint16_t temTime;
  static uint8_t k = 3, lcd_recover = 1;   //for lcd k for animation  lcd_recover should be greater than 120 initially for lcd.begin to be called in lcdmenu().
  static unsigned long millisLastLCD = 0;  //update tracker for the LCD to avoid flicker

  if (option == LCD_Display) {  // this must be only for display other need prompt response

    if (millisCurrent - millisLastLCD < interval) {  // working

      return;

    } else {

      switch (k++) {  //little animation

        case 0:  //.5s
        case 2:
          lcdPos(0, 7);
          lcd.print(F(" "));
          //lcd.createChar(7, Char_AA[0]);
          millisLastLCD = millisCurrent;
          return;  //causes the lcd to update each as it cause return every .5 sec
        case 1:
          lcd.createChar(7, Char_AA[0]);
          break;
        case 3:
          lcd.createChar(7, Char_AA[1]);
          k = 0;
          break;
      }

      if ((lcd_recover++ % 60) == 0) {       //reset lcd every 1 min 60*1s=1min
        lcd.begin(col, row);                 //to prevent the grabelling after glitch after with lcd is in byte mode not in nibble as set up by LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
        lcd.createChar(1, Char_centigrade);  //byte(0) Char_centigrade
        lcd.createChar(2, Char_Lux);
        lcd.createChar(3, Char_PH);
        //lcd.createChar(4, Char_ArrowRight);  //byte(1) Char_ArrowRight
        lcd.createChar(5, Char_F1);
        lcd.createChar(6, Char_F2);
        lcd_recover = 1;
      }
    }
  }

  switch (option) {

    case LCD_Demo:

      #ifdef __DEBUG_LCD_DEMO_
        lcd.clear();
        lcd.print(F("Contrast:"));
        for (int i = 0; i < 150; i++) {
          analogWrite(contrastPin, i);
          lcd.setCursor(0, 1);
          lcd.print(i);
          delay(80);
        }

        analogWrite(contrastPin, 50);
        lcd.clear();
        lcd.print(F("Back light:"));
        for (int i = 0; i < 255; i += 2) {
          analogWrite(backlightPin, i);
          lcd.setCursor(0, 1);
          lcd.print(i);
          delay(80);
        }

        analogWrite(contrastPin, contrast_val);    //set some contrast
        analogWrite(backlightPin, backlight_val);  //set backlight on

      #endif

      break;

    case setupEEPROM:  // this should be blocking
      lcd.clear();
      lcdLnClr(1);
      lcd.print(F("Using Defaults"));
      delay(1000);
      // inputParameters();
      // break;

    case Update_EEPROM:
      inputParameters();
      break;

    case LCD_Display:  // update tweak for layout formating when all is finished

      currentTemp = getCelsius(insideThermometer);  //deci scale
      currentLight = getLDR();                      //ok  // normal scale
      current_pH = getPH();                         //ok  set ph into deci


      lcdLnClr(0);
      lcd.print(F("Now  "));
      lcdTimePrint(time_hh);
      lcd.print(F(":"));
      lcdTimePrint(time_min);
      lcd.print(F("  Set"));

      lcdLnClr(1);
      lcd.write(1);  //degree centi

      lcd.print(currentTemp / 10.0, 1);

      lcd.setCursor(6, 1);  //possible bug here
      lcd.write(7);
      lcd.write(7);
      lcd.write(7);
      lcd.write(7);
      lcd.write(7);

      lcd.setCursor(12, 1);
      lcd.print(x.setTemp / 10.0, 1);

      lcdLnClr(2);
      lcd.write(2);  //lux
      lcd.print(currentLight);

      lcdPos(2, 6);  //possible bug here
      lcd.write(7);
      lcd.write(7);
      lcd.write(7);
      lcd.write(7);
      lcd.write(7);

      lcdPos(2, 12);
      lcd.print(x.setLux);

      lcdLnClr(3);
      lcd.write(3);  //ph

      lcd.print(current_pH / 10.0, 1);

      lcdPos(3, 5);
      lcd.write(5);  //F1

      lcdTimePrint(x.feedTime1 / 60);
      lcdTimePrint(x.feedTime1 % 60);

      lcdPos(3, 11);
      lcd.write(6);  //F2

      lcdTimePrint(x.feedTime2 / 60);
      lcdTimePrint(x.feedTime2 % 60);
      millisLastLCD = millisCurrent;
      break;
  }



}  //lcdmenu() // need lcd update code from here to the input parameters

void f_setTemp() {  //blocking

  unsigned long millisLastParameters = millis();
  uint8_t temp = x.setTemp;  //too detect the change
  lcd.clear();
  if (x.setTemp < 190 || x.setTemp > 325) {  //should never execute except some fault
    x.setTemp = 280;
    saveToEEPROM();
  }
  while (1) {  // infinite loop blocking
    lcdLnClr(1);
    lcd.print(F("Temp Setting"));
    lcdLnClr(2, 4);
    lcd.print(x.setTemp / 10.0, 1);
    lcd.write(1);  //lcd line 2 and 3 puts tabs at intial so must be select -ve column

    switch (Rot_Encoder_Poll()) {

      case NO_CHANGE:
        if (millis() - millisLastParameters > 2000) {  //time out 2 sec
          retriveFromEEPROM();
          return;  //read  variables from eeprom if 0 to reverse the changes introduced  //reverse change not implemnted yet
        }

      case CLOCKWISE:
        if (x.setTemp < 325)
          x.setTemp += 5;

        break;

      case ANTICLOCKWISE:
        if (x.setTemp > 190)
          x.setTemp -= 5;

        break;

      case SW_BUTTON:

        saveToEEPROM();
        return;
    }

    millisLastParameters = millis();
  }

}  //f_setTemp()

void f_setLux() {  //blocking

  unsigned long millisLastParameters = millis();
  lcd.clear();
  while (1) {  // infinite loop blocking
    lcdLnClr(1);
    lcd.print(F("Light Setting"));
    lcdLnClr(2, 4);
    lcd.print(x.setLux);

    switch (Rot_Encoder_Poll()) {

      case NO_CHANGE:
        if (millis() - millisLastParameters > 2000) {  //time out 2 sec
          retriveFromEEPROM();
          return;  //read  variables from eeprom if 0 to reverse the changes introduced  //reverse change not implemnted yet
        }
        break;

      case CLOCKWISE:
        if (x.setLux < 1010)
          x.setLux += 10;

        millisLastParameters = millis();
        break;

      case ANTICLOCKWISE:
        if (x.setLux > 40)
          x.setLux -= 10;

        millisLastParameters = millis();
        break;

      case SW_BUTTON:

        saveToEEPROM();
        return;
    }
  }
}  //f_setLux()

void f_setTime(byte sel) {

  String MenuTime_message[2] = {
    "Set hh",
    "Set mm",
  };

  unsigned long millisLastParameters = millis();

  lcd.clear();
  byte selector = 0;  //0 for hh 1 for min
  String str[3] = { "Clock", "FeedTime1", "FeedTime2" };

  switch (sel) {  //prelude
    case 0:

      rtc_now = rtc.now();
      time_hh = rtc_now.hour();
      time_min = rtc_now.minute();
      break;

    case 1:

      time_hh = x.feedTime1 / 60;
      time_min = x.feedTime1 % 60;
      break;

    case 2:

      time_hh = x.feedTime2 / 60;
      time_min = x.feedTime2 % 60;
      break;

    default:

      break;

  }  //end of the prelude to the setting times

  while (1) {  //interlude

    lcdLnClr(0, 5);
    lcd.print(str[sel]);
    while (selector == 0) {

      lcdLnClr(1);
      lcd.print(F("set HH (24)"));
      lcdLnClr(2, 6);
      if (time_hh < 10) {
        lcd.print(F("0"));
      }
      lcd.print(time_hh);

      switch (Rot_Encoder_Poll()) {

        case NO_CHANGE:
          if (millis() - millisLastParameters > 2000) {  //time out 2 sec

            return;
          }  //read  variables from eeprom if 0 to reverse the changes introduced
          break;

        case CLOCKWISE:

          time_hh++;
          if (time_hh > 23) {
            time_hh = 0;
          }
          millisLastParameters = millis();
          break;

        case ANTICLOCKWISE:

          if (time_hh > 0) {
            time_hh--;
          } else {
            time_hh = 23;
          }
          millisLastParameters = millis();
          break;

        case SW_BUTTON:

          selector = 1;  //save  variables to eeprom if 1 to record the changes introduced
      }
    }

    millisLastParameters = millis();

    while (selector == 1) {

      lcdLnClr(1, 4);
      lcd.print(F("set mm"));
      lcdLnClr(2, 6);

      if (time_min < 10) {
        lcd.print(F("0"));
      }
      lcd.print(time_min);

      switch (Rot_Encoder_Poll()) {

        case NO_CHANGE:
          if (millis() - millisLastParameters > 2000) {  //time out 2 sec

            return;
          }
          break;

        case CLOCKWISE:

          time_min++;
          if (time_min > 59) {
            time_min = 0;
          }
          millisLastParameters = millis();
          break;

        case ANTICLOCKWISE:

          if (time_min > 0) {
            time_min--;
          } else {
            time_min = 59;
          }
          millisLastParameters = millis();
          break;

        case SW_BUTTON:

          selector = 2;
          millisLastParameters = millis();
          break;  //save  variables to eeprom if 1 to record the changes introduced
      }
    }

    if (selector = 2) {

      switch (sel) {

        case 0:

          rtc_now = rtc.now();

          rtc.adjust(DateTime(rtc_now.year(), rtc_now.month(), rtc_now.day(), time_hh, time_min, rtc_now.second()));  //adjusting time working

          rtc_now = rtc.now();
          return;  //no need to update rom
          break;

        case 1:
          x.feedTime1 = time_hh * 60 + time_min;
          state.FoodServe1 = LOW;

          saveToEEPROM();  // changes saved feedtime1 updated
          return;
          break;

        case 2:
          x.feedTime2 = time_hh * 60 + time_min;
          state.FoodServe2 = LOW;

          saveToEEPROM();  //changes saved feedtime2 updated
          return;          //no need to update eeprome
          break;

        default:

          return;
      }
    }
  }
}  //f_setTime(byte sel)

void inputParameters() {  //blocking

  uint8_t menuTop = 0;  //for tracking the displayed menu
  uint8_t menuPosition = 0;
  uint8_t menuSubPosition = 0;
  lcd.createChar(4, Char_ArrowRight);  //byte(1) Char_ArrowRight

  const uint8_t NumMenu = 6;
  String MenuLvl1_message[NumMenu] = {
    "Set Temp",       //0
    "Set lux",        //1
    "Set Clock",      //2
    "Set FeedTime1",  //3
    "Set FeedTime2",  //4
    "Save/Exit"       //5
  };

  uint8_t whileFlag = 1;
  unsigned long millisLastInputParameters = 0;

  while (whileFlag) {

    millisLastInputParameters = millis();
    lcd.clear();

    if (menuPosition >= NumMenu)
      menuPosition--;

    for (uint8_t i = menuTop, j = 0; i < menuTop + 4; i++, j++) {  //mem smashing occuring here //fixed

      lcdLnClr(j, 1);
      if (i == menuPosition) {
        lcd.write(4);
      }  //print right arrow
      else {
        lcd.print(' ');
      }
      lcd.print(MenuLvl1_message[i]);
    }

    switch (Rot_Encoder_Poll()) {  // unprermitted memory access problem posssibly interplay of menuposition and menutop

      case NO_CHANGE:
        if (millis() - millisLastInputParameters > 3000)  //

          whileFlag = 0;
        break;

      case CLOCKWISE:
        if (menuPosition < NumMenu - 1)  // bug here
          menuPosition++;
        if (menuPosition >= 4 && menuPosition < NumMenu)  //scrolls
          menuTop++;
        if ((menuTop + 4) > (NumMenu))  //this fix the issue but need to understand why // here 4 is the no of line in the lcd  //fixed
          menuTop--;
        millisLastInputParameters = millis();  // reset timeout
        break;

      case ANTICLOCKWISE:  //this is working perfectly
        if (menuPosition > 0)
          menuPosition--;
        if (menuPosition <= NumMenu - 1 && menuTop > 0)
          menuTop--;
        millisLastInputParameters = millis();  // reset timeout
        break;

      case SW_BUTTON:

        switch (menuPosition) {

          case 0:
            f_setTemp();
            break;

          case 1:
            f_setLux();
            break;

          case 2:
            f_setTime(0);
            break;

          case 3:
            f_setTime(1);
            break;

          case 4:
            f_setTime(2);
            break;

          case 5:
            whileFlag = 0;
            break;

          default:

            break;
        }

        break;
    }
  }

}  //inputParameters()
