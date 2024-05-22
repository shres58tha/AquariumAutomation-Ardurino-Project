#define abs(x) ((x) > 0 ? (x) : -(x))
//needed for the signed value only unsigned cannot yield negative value so this is useless ie (x) always greater than 0 soo is (x) itself
// STRIPPED FILE ]
#define default_val {0x0,0xE6,0x0,0xC2,0x1,0x1C,0x2,0x38,0x4} 
//Debug

//  #define __SERIAL.BEGIN__
//  #define __DEBUG__TEMP_
//  #define __DEBUG__ROT_
//  #define __DEBUG__LUX_
//  #define __DEBUG__PH_
//  #define __DEBUG__MB_
//  #define __DEBUG__BLINK_
//  #define __DEBUG__EEPROM_
//  #define __DEBUG__CTRL_
//  #define __DEBUG__SETUP_
//  #define __DEBUG__LOOP_
//  #define __DEBUG__LCDMENU_
//  #define __DEBUG_LCD_DEMO_
//  #define __DEBUG__f_setTemp_
//  #define __DEBUG__f_setLux_
//  #define __DEBUG__f_setTime_
//  #define __DEBUG__f_setTime_setClock_
//  #define __DEBUG__f_inputParameters_
//  #define __DEBUG__feed_
//  #define __DEBUG__servo_
//  #define __DEBUG__ALARM_
//  #define __INTIAL_TIME_ESTIMATION_
//  // initial time estimation can be used to set the clock accurately
//  #define __LOOP_TIME_ESTIMATION_
//  #define _SoundALarm_ON_

  #define __pH_Meter__PRESENT_
  #define cal_val 9.7

#include <digitalWriteFast.h>

  // pinModeFast (pinNum)
  // digitalWriteFast(pinNum, state) (sets or* clears pin/port faster)
  // pinModeFast(pinNum, mode) (sets pin/port as input or output faster)
  // digitalReadFast(pinNum)(reads the state of pin/port faster)
  // digitalToggleFast(pinNum)(toggles the state of pin/port faster)
  

#ifndef MENU_H  //Liquid Crystal menu
#define MENU_H

  #include <LiquidCrystal.h>
  //declare the lcd 2004 20 * 4 pins declarations
  //pin name        1   2   3   4   5   6  7    8   9   10  11  12  13  14   15         16
  //signal          VSS VDD V0  RS  R/W E  DB0  DB1 DB2 DB3 DB4 DB5 DB6 DB7  LED+       LED-
  //value rt to lf  gnd  5v pot      5v     na  na  na  na                   10kReg5V   gnd
  const uint8_t rs = 27, en = 26, d4 = 25, d5 = 24, d6 = 23, d7 = 22;
  const uint8_t col = 20, row = 4;
  LiquidCrystal lcd(rs, en, d4, d5, d6, d7);  // object creation
                                              //LiquidCrystal  lcd (27, 26, 25, 24, 23, 22);		// object creation
  #define contrastPin 11                      //pwm capable pin //not necessary for 1604 display must for 2004 display
  #define backlightPin 12                     //pwm capable pin
  uint8_t contrast_val = 200;                 //does not work in this lcd 
  uint8_t backlight_val = 50;                 //can be used to dim the backlight as per light measurement backlight = (currentLight/5) + 15 


  enum lcd_option { DoNothing,
                    LCD_Demo,
                    setupEEPROM,
                    Update_EEPROM,
                    LCD_Display };
  lcd_option lcdOptOld_val = DoNothing;
  void lcdmenu(lcd_option option);                    //declaration of lcdmenu()
  void lcdLnClrPos(uint8_t lineNo, uint8_t pos = 0);  //declaration of lcdLnClr() small function to clear line and set postition of cursor to pos handles funny LCDd

#endif

//rotary encoder with button + 1 button input + 1 button feed interrupt
  #define Rot_Encoder_CLK 28
  #define Rot_Encoder_DT 29
  #define Rot_Encoder_SW 30         //pressed when low
  #define Mode_Button 31            //use internal pull up register for const when pressed low
  #define Feed_Button 2             //it works here? INPUT_PULLUP used  active when low

enum Rot_Encoder_Event { NO_CHANGE,
                         CLOCKWISE,
                         ANTICLOCKWISE,
                         SW_BUTTON };
//String Rot_Encoder_Event_String[5] {"NO_CHANGE", "CLOCKWISE", "ANTICLOCKWISE", "SW_BUTTON"};    //not used anywhere  can be commented out

//function declarations
  uint16_t getLux();
  uint8_t getPH();
  Rot_Encoder_Event Rot_Encoder_Poll();
  bool MB_Event();
  void alarm(bool state);
  void Blink(uint16_t blinkDuration);
  void retriveFromEEPROM();
  void saveToEEPROM();
  void ReadCurrents();
  void ControlState();
  void setup();
  void loop();
  void lcdLnClrPos(uint8_t lineNo, uint8_t pos);
  void lcdmenu(lcd_option option);
  void f_setTemp();   // need to save
  void f_setLux();
  void f_setTime(byte sel);
  void inputParameters();



//sensor ingroup
  #define ONE_WIRE_BUS 10   //pin to read for temp sensor  digital one wire pin
  #define currentPinLux A8  //use internal pullup resistor pin to read form ldr  valid
  #define currentPin_pH A9  //pin to read from pH meter // analog valid
  #define currentT0_PH 00

//rotary encoder done

#include <RTClib.h>
RTC_DS3231 rtc;  //uses tx rx  I2C channel autosearch mode pin used 14 15
DateTime rtc_now;
int16_t checkFeedTime=0;
byte time_hh ; 
byte time_min;

#if defined __DEBUG__SETUP_ || defined __DEBUG__f_setTime_setClock_
    char daysOfTheWeek[7][4] = { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };
#endif


// temp int rct clock //ph sensors // temp in ph sensor too
// Thermometer
#include <OneWire.h>
#include <DallasTemperature.h>
//onewire object creation for thermometer
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature oneWireSensors(&oneWire);
DeviceAddress insideThermometer;  // array to hold address inside thermometer
uint16_t getCelsius(DeviceAddress deviceAddress);

#include <stdint.h>
#include <EEPROM.h>
/*EEPROM Example
    EEPROM.read() – Read a byte from EEPROM.
    EEPROM.write() – Write a byte to EEPROM.
    EEPROM.update() – Write a byte to EEPROM only if the current value is different to previous value.
    EEPROM.get() – Read any datatype from EEPROM (float or struct or any datatype).
    EEPROM.put() – Write any datatype to EEPROM (float or struct or any datatype)
*/

#include <Servo.h>
//servoFeed object created in setup and attached and detached to pin in loop. only call from loop is working function wraping not possible


//for the dallas thermometer  possible alu

  #ifdef __DEBUG__TEMP_
    void printAddress(DeviceAddress deviceAddress) {
      for (uint8_t i = 0; i < 8; i++) {
        if (deviceAddress[i] < 16) Serial.print(F("0"));
        Serial.print(deviceAddress[i], HEX);
      }
      delay(100);
    }
  #endif

//function commences 

uint16_t getCelsius(DeviceAddress deviceAddress) {  //working  dont touch

  #ifdef __DEBUG__TEMP_
    Serial.println(F("Before blocking requestForConversion"));
    unsigned long start = millis();
  #endif

  oneWireSensors.requestTemperatures();


  #ifdef __DEBUG__TEMP_
    unsigned long stop = millis();
    Serial.println(F("After blocking requestForConversion"));
    Serial.print(F("Time used:"));
    Serial.println(stop - start);
  #endif

  uint16_t tempC = 10 * oneWireSensors.getTempCByIndex(0);  //first tempSensor


  #ifdef __DEBUG__TEMP_
      //Serial.println(' ');
    Serial.print(F("\nTemperature read from the thermometer"));
    Serial.println(tempC, 1);
  #endif

  return tempC;
}


//for the LDR
uint16_t getLux() {  // returns Lux normal scale
  /***************** INFORMATION
      Typically 
      RL=500/lux
      V0=5*(RL/(RL+R))
      V0=LDR_value*ADC_value   
      lux=((2500/V0)-500)/R
      https://emant.com/316002
      https://www.allaboutcircuits.com/projects/design-a-luxmeter-using-a-light-dependent-resistor/

      //to prevent short circuit when LDR is highly illluminated
    [GND]--R[10k ohm resistor]--| -- [LDR] -- [5V]
                                |
                            [Analog Input A0]
  Different equation here but better is to calibrate (curve fixing by correlation) taking mobile sensor as reference

  Condition	    Illumination(lux) analog reading of current LDR
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

  #ifdef __DEBUG__LUX_
    uint16_t lux = 0;  //ADC = 1/1024
    //pinModeFast(currentPinLux ,INPUT);
    uint8_t n = 3;

    for (uint8_t i = 0; i < n; i++) {  // taking average of n
      lux += analogRead(currentPinLux);
      delay(10);  //wait 10 micros
    }
    lux = lux / n;  //lux=((2500*n/lux)-500)/10000; //scaling  lux in float here but not done in this case
    Serial.print(F("lux:"));
    Serial.println(lux);  // delay(2000);
    Serial.print(F("NOTE: IN THE ACTUAL FUNCTION  return analogRead(currentPinLux)  is used "));

    return lux;

  #endif

  #ifndef __DEBUG__LUX_
      return analogRead(currentPinLux);
  #endif
}


// for pH meter https://circuitdigest.com/microcontroller-projects/arduino-ph-meter
//  for(int i=2;i<6;i++)
//  avgval+=buffer_arr[i];
//  float volt=(float)avgval*5.0/1024/6;
//  float ph_act = -5.70 * volt + calibration_value;

/*
uint8_t getPH() {  //returns in pH*10    //returns deci scale


  #ifdef __DEBUG__PH_

    float tmp = 0, ana_pH;
    uint8_t n = 4;

    for (int i = 0; i < n; i++) {
      tmp += analogRead(currentPin_pH);
      delay(10);
    }

    float avgval = tmp / n;
    float volt = (float)avgval * 5.0 / 1024;
    ana_pH = -5.70 * volt + 21.34 + cal_val;

  
    Serial.print(F("volt:"));
    Serial.println(volt);
    Serial.print(F("after ana_pH:"));
    Serial.println(ana_pH);
    Serial.print(F("NOTE: IN THE ACTUAL FUNCTION  return analogRead(currentPin_pH)*bla bla  is used "));
    delay(1000);
    return (uint8_t) 10 * ana_pH;
  #endif

  #ifndef __DEBUG__PH_

    #ifdef  __pH_Meter__PRESENT_

      #ifdef __SERIAL.BEGIN__
        float tmpH= 0;
        for (int i = 0; i < 5; i++) { //here 5 is taken into consideration while doing following calculation
          tmpH += analogRead(currentPin_pH);
          delay(10);
        }      
    
        tmpH = 317 - 57*tmpH/1024;  //float avgval = tmp / n; float volt = (float)avgval * 5.0 / 1024; ana_pH = -5.70 * volt + 21.34 + cal_val
        Serial.print(F("tmpH:"));        
        Serial.println(tmpH);
        return tmpH;
      #else
            return 317 - 57*analogRead(currentPin_pH)*5/1024;  //cal_val  -5.7*10*(tmpH/5)*(5/1024) 
      #endif
        
    #else
      return 70;
    #endif      
  #endif
}*/
uint8_t getPH() {  //returns in pH*10    //returns deci scale  //rolling average  //non blocking and avioids sudden jumps
  #ifdef  __pH_Meter__PRESENT_
      const uint8_t size = 10;
      static float pHs[size], sum=0;  //static are by default initialized 0
      static uint8_t i=0;
      float val = (21.34 + cal_val) - 5.7*analogRead(currentPin_pH)*5/1024;
      if (i<size){ 
        pHs[i]=val;
        i++;
        sum +=val;
      } 
      else{  //shift
        sum-=pHs[0];
        for ( int j = 0; j<size-1; j++){
          pHs[j]=pHs[j+1];           //pHs[last-1] is freed
        }
        pHs[size -1]=val;
        sum+=val;
      }
      return sum;  //in deci. scale
  #else
    return 70;
  #endif 
}
  


//GLobal Variables
//servo limits
  #define servoMinDegrees 85  //the limits to servo movement
  #define servoMaxDegrees 130
  #define feedtimes 3
  uint8_t feedserved=0;

//Servo  constants
// _DEBUG__SERVO_
Servo servoFeed;                        //create servo object to control a servo 
uint8_t servoPosition;  //the intial servoPosition of the servo - starting at 0.


struct {        
   //initialized variable                       //use bit fields then initilization must be done later cant be done on the structure like below
  bool FoodServe = LOW;
  bool FoodServe1 = LOW;              //reset these at  control at hhmm 0000 or hhmm=0;
  bool FoodServe2 = LOW;
  bool readFlag = LOW;                //1 indicate that the changes to be updated in the EEPROM
  bool FilterCtrlPin = LOW;           // normal state is low --> filter pump on
  bool SW_Current = HIGH;
  bool SW_Last = HIGH;
  bool MB_Current = HIGH;
  bool MB_Last = HIGH;                 //is high when button is not pressed
  bool RE_CLK_Current = LOW;           //rotary encoder
  bool RE_CLK_Last = LOW; 
} state;

// union {
//   struct {
//     //initialized variable//use bit fields then initilization must be done later cant be done on the structure like below
//     bool HeaterCtrlPin : 1;  //normal state is low--> heater is on
//     bool LightCtrlPin : 1;
//     bool FoodServe : 1;
//     bool FoodServe1 : 1;  //reset these atcontrol at hhmm 0000 or hhmm=0;
//     bool FoodServe2 : 1;
//     bool readFlag : 1;       //1 indicate that the changes to be updated in the EEPROM
//     bool FilterCtrlPin : 1;  // normal state is low --> filter pump on
//     bool SW_Current : 1;
//     bool SW_Last : 1;
//     bool MB_Current : 1;
//     bool MB_Last : 1;         //is high when button is not pressed
//     bool RE_CLK_Current : 1;  //rotary encoder
//     bool RE_CLK_Last : 1;
//   };
//   byte data[2] ={0x80, 0x07};
// } state;


//variables structure x
union {
  struct {
    byte flag;               //1 bool     valid data means flag 1
    uint16_t setTemp;        //2 byte     in temp*10 scale
    uint16_t setLux;         //2 byte     int lux*10 scale
    uint16_t feedTime1;      //2 byte  
    uint16_t feedTime2;      //2 byte     in minutes  hh*60+mm            //overwritten by one eeprom
  };                         //9 bytes
  byte byteData[9]=default_val;  //default value          //using as pointer to above data structure need to use sizeof to get the byte lenght while processing
} x;

// working variable here
  //uint8_t range_pH = 12;   //+-15 ie ph 5.8 to 8.2  deci scale  
  #define range_pH 12 
  uint8_t current_pH;        // use deci range 0 to 140 val*10
  uint8_t normal_pH_7 = 70;  //used deci
  uint16_t currentTemp;      // use deci for both in val*10 scale  Range 10 unit
  uint16_t currentLight;     // uint16_t to accomodate 1024 levels the output of ADC  2 bytes

  
  uint8_t k = 0, lcd_recover = 120; //for lcd k for animation  lcd_recover should be greater than 120 initially for lcd.begin to be called in lcdmenu().
 
//time working variables
  unsigned long millisCurrent=0;        //stores the value of millis() in each iteration of loop()
  unsigned long millisLastRead=0;       //will store last time anlog input were read
 
  #ifdef __LOOP_TIME_ESTIMATION_
    unsigned long millisEndofLoop = 0;
  #endif

  unsigned long millisFilterStartCommence = 0;      //for keeping track of filter pump off period
  unsigned long millisFilterPumpOffTime = 1800000;  //turn pump off for half hr in case food is delivered
  unsigned long millisLastBoardLed = 0;             //last board blink in millis
  unsigned long millisLastLCD = 0;                  //update tracker for the LCD to avoid flicker
  uint16_t interval = 500 ;                         //number of millisecs between blinks lcd and read in millis
 
// unsigned long millisOverflow = 0 ;							  // not needed  can be used to extend millies counter to 64 bit


// custom characters
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
byte Char_AA[2][8]={
  {
    B00010,
    B00101,
    B00111,
    B00101,
    B01000,
    B10100,
    B11100,
    B10100
  },
  {
    B01000,
    B10100,
    B11100,
    B10100,
    B00010,
    B00101,
    B00111,
    B00101
  }
};


// https://maxpromer.github.io/LCD-Character-Creator/


Rot_Encoder_Event Rot_Encoder_Poll() {
  static uint8_t counter = 0;
  unsigned long millisLocal_SW;
  unsigned long millisTimeout = 4000;
  unsigned long millisInitial = millis();
  while (1) {
    millisLocal_SW = millis();


  #ifdef __DEBUG__ROT_
      Serial.println(F("IN Rot_Encoder_Poll"));
  #endif


  state.RE_CLK_Current = digitalReadFast(Rot_Encoder_CLK);
  if (state.RE_CLK_Current != state.RE_CLK_Last) {  // edge
    state.RE_CLK_Last = state.RE_CLK_Current;
    if (!state.RE_CLK_Current) {              //falling edge
      if (digitalReadFast(Rot_Encoder_DT)) {  // if high rotating anticlockwise

        #ifdef __DEBUG__ROT_
                  counter++;
                  Serial.print(F("Position :"));
                  Serial.println(++counter);
        #endif

        return ANTICLOCKWISE;

      } else {

        #ifdef __DEBUG__ROT_
                  counter--;
                  Serial.print(F("Position :"));
                  Serial.println(--counter);
        #endif

        return CLOCKWISE;
      }
    }
  }
  
  //  state.RE_CLK_Last = RE_CLK_Current; // Updates the previous state of the Rot_Encoder_CLK with the current state
  state.SW_Current = digitalReadFast(Rot_Encoder_SW);
  if (state.SW_Last != state.SW_Current) {  // detecting falling edge
    state.SW_Last = state.SW_Current;
    if (!state.SW_Current) {


      #ifdef __DEBUG__ROT_
              Serial.println(F("Falling edge of button  :"));
      #endif

      delay(20);
      return SW_BUTTON;
    }
  }
    
  if (millisLocal_SW - millisInitial > millisTimeout)  //return if no action for 100  effectively making it non blocking
    return NO_CHANGE;
  delay(5);

  }

} // Rot_Encoder_Poll()


bool MB_Event() {                               //Mode Button Event
  state.MB_Current = digitalRead(Mode_Button);  //normal is the high
  if (state.MB_Last != state.MB_Current) {      // detecting falling edge

      #ifdef __DEBUG__MB_
          Serial.print(F("state.MB_Last"));
          Serial.println(state.MB_Last);
          Serial.print(F("state.MB_Current"));
          Serial.println(state.MB_Current);
      #endif

      state.MB_Last = state.MB_Current;
      if (!state.MB_Current) {  //if state.MB_Currnt = 0

        #ifdef __DEBUG__MB_
              Serial.println(F("Falling edge of Mode_Button  :"));
        #endif

        return 1;

      } else {
          return 0;
      }

  } else {
    state.MB_Last = state.MB_Current;
  }

  return 0;
} //MB_Event()

//output pin group
  #define controlPinUnknown 32   // notworking cut off just wire available outside
  #define controlPinLighting 33  //for controlling aquarium light normal is low.
  #define controlPinHeater 34     // heater default state low  while high cutoff the heater
  #define controlPinFilter 35     // default state low high cutoff
  #define controlPinAlarm 36      //for the alarm indicatro led and or buzzer
  #define controlPinAlarmLED 37  
  #define controlPinFoodServo 8  //pwn  needed servo motor turn pin. Normal low. introduce feed when high


  #define onBoardLedPin 13  //onBoard LED used to indicate operation state by blinking every x loops

//

void alarm(bool state) {  //works properly
  //state=LOW;


  #ifdef __DEBUG__ALARM_
      Serial.print("alarmState:");
      Serial.println(state);  //entering here


    if (state) {
      Serial.println(F("Alarm Blaring"));
    }
  #endif

  digitalWriteFast(controlPinAlarmLED,state);

  #ifdef _SoundALarm_ON_
    digitalWriteFast(controlPinAlarm, state);
  #endif

} //alarm(bool state) 

void Blink(uint16_t blinkDuration ) {

  #ifdef __DEBUG__BLINK_
    Serial.print(millisCurrent);
    Serial.println(F("Blink Blink"));
  #endif

  if (millisCurrent - millisLastBoardLed >= blinkDuration) {
    digitalToggleFast(onBoardLedPin);  //real Toggle herein
    //digitalToggleFast(indicatorLedPin);
    millisLastBoardLed += blinkDuration;  //increment the millis
  }
} //Blink(unsigned long blinkDuration)


void retriveFromEEPROM() {


  #ifdef __DEBUG__EEPROM_
  //Serial.println( F("Retriving from EEPROM") );
  #endif

  if ( EEPROM.read(0)==1) {  //first address is 0   //valid data is from byte 1
    for (uint8_t i =1 ; i < sizeof(x); i++) {
      x.byteData[i] = EEPROM.read(i);


  #ifdef __DEBUG__EEPROM_
        Serial.println(' ');
        Serial.print(F("x.byteData"));
        Serial.println(i);
        Serial.print(F(":"));
        Serial.println(x.byteData[i]);
  #endif

    }
  }
  else {
     lcdmenu(setupEEPROM);  
  }


  #ifdef __DEBUG__EEPROM_
    delay(1000);
  #endif

} //retriveFromEEPROM()

/*
  customVar = (MyObject){ "sth", "next", 10298763, "  #101  #23"};
  int eeAddress = 0;
  EEPROM.put(eeAddress, customVar);
  */

void saveToEEPROM() {
  x.flag = 1;
  for (uint8_t i = 0; i < sizeof(x); i++) {
    EEPROM.update(i, x.byteData[i]);
  }
} //saveToEEPROM()


void ReadCurrents() {  //need updating

  if (millisCurrent - millisLastRead >= interval) {
    currentTemp = getCelsius(insideThermometer);  //deci scale
    currentLight = getLux();                        //ok  // normal scale
    current_pH = getPH();                         //ok  set ph into deci
    millisLastRead = millisCurrent;                    //putting most recent value of millis in last read one.
  }
} //ReadCurrents()


void ControlState() {  //need debug here check the outputs  //nonblocking
  bool flag_alarm=0;

  #ifdef __DEBUG__CTRL_
    Serial.println(F("Control State"));  //entering here
  #endif


  //heater control

  #ifdef __DEBUG__CTRL_
    Serial.print(F("Heater:"));  //entering here
  #endif

  if (currentTemp < x.setTemp - 5){ //turn on at .5 degree lower than set
     digitalWriteFast(controlPinHeater, LOW);  //becomes LOW  heater on
      #ifdef __DEBUG__CTRL_
          Serial.println(F("controlPinHeater LOW"));  //entering here
      #endif
    }
  if (currentTemp >= x.setTemp ){  //hysterisis loop included to stop frequent on and off . 
     digitalWriteFast(controlPinHeater, HIGH);  //becomes HIGH   heater off
     #ifdef __DEBUG__CTRL_
          Serial.println(F("controlPinHeater HIGH"));  //entering here
      #endif
  }


 

  //Light control

  #ifdef __DEBUG__CTRL_
    Serial.print(F("Light:"));  //entering here
  #endif

  if (currentLight < x.setLux){   //introducing hysteresis to prevent flickering
     digitalWriteFast(controlPinLighting, HIGH);  //becomes high Light on
      #ifdef __DEBUG__CTRL_
           Serial.println(F("controlPinLighting, HIGH"));  //entering here
      #endif
      digitalWriteFast(onBoardLedPin, LOW);  //turn off LED
  }
  if (currentLight > x.setLux + 100){   
     Blink(interval);  
     digitalWriteFast(controlPinLighting, LOW);  //becomes low lights off
      #ifdef __DEBUG__CTRL_
           Serial.println(F("controlPinLighting, LOW"));  //entering here
     #endif
  }
  


  #ifdef __DEBUG__CTRL_
    Serial.println();  //entering here
  #endif


  //pH temperature range alarm

  #ifdef __DEBUG__CTRL_
    Serial.print(F("pH °C alarm ->"));  //entering here
  #endif

  

  
  if (abs(current_pH - normal_pH_7) > 15 ){  //evaluated value can be negetive

    #ifdef __DEBUG__ALARM_
        Serial.println("Alarm due pH");
        Serial.print("current_pH : ");
        Serial.println(current_pH);  
        Serial.print("normal_pH_7 : ");
        Serial.println(normal_pH_7);  
    #endif

      flag_alarm=1;
  }


  if (x.setTemp > currentTemp + 20) {  //stored units are decis ie 20.0 stored as 200 since it is unsigned the diff is always positi

    #ifdef __DEBUG__ALARM_
        Serial.println("Alarm due low Temp");
        Serial.print("currentTemp : ");
        Serial.println(currentTemp);
        Serial.print("x.setTemp : ");
        Serial.println(x.setTemp);
    #endif

      flag_alarm=1; 
  }    
  

  if (currentTemp > x.setTemp + 20) {  //stored units are decis ie 20.0 stored as 200

    #ifdef __DEBUG__ALARM_
        Serial.println("Alarm due  High Temp");
        Serial.print("currentTemp : ");
        Serial.println(currentTemp);
        Serial.print("x.setTemp : ");
        Serial.println(x.setTemp);
    #endif

      flag_alarm=1; 
  }  

  #ifdef __DEBUG__ALARM_    
      Serial.print("flag_alarm : ");
      Serial.println(flag_alarm);
  #endif 

  if (flag_alarm){
    alarm(HIGH);
  } else {
    alarm(LOW);
  }
  #ifdef __DEBUG__CTRL_    
        Serial.println(flag_alarm);
  #endif 

  //food control

  ////to make truely non blocking call oneself here with flag preventing entry to this block of code feed_block
  //feed_block 
    #ifdef __DEBUG__CTRL_
      Serial.print(F("Feed ->"));  //entering here
    #endif


    #ifdef __DEBUG__feed_
      Serial.println (F("Feed ->"));  //entering here
      Serial.print("state.FoodServe1 :");Serial.println(state.FoodServe1);
      Serial.print("state.FoodServe2 :");Serial.println(state.FoodServe2);    
      Serial.print("x.feedTime1 :");Serial.println(x.feedTime1);
      Serial.print("x.feedTime2 :");Serial.println(x.feedTime2);
      Serial.print("checkFeedTime :");Serial.println(checkFeedTime);    
    #endif

    if (checkFeedTime == x.feedTime1) {
      if (state.FoodServe1 == LOW) {  //pin low
        state.FoodServe = HIGH;      //becomes high servo on 
        state.FoodServe1= HIGH; 

        #ifdef __DEBUG__feed_
              Serial.print(F("FeedTime1  : HIGH"));  //entering here
              delay(100);
        #endif
      
      }
        
    }
    else{
       state.FoodServe1 = LOW;      
       #ifdef __DEBUG__CTRL_
        Serial.println(F("Reset FoodServe1 : LOW"));  //entering here
       #endif
    }

    if (checkFeedTime == x.feedTime2) {
      if (state.FoodServe2 == LOW) { 
        state.FoodServe = HIGH;      //pin high food served
        state.FoodServe2= HIGH; 

        #ifdef __DEBUG__feed_
              Serial.print(F("FeedTime2  : HIGH"));  //entering here
              delay(100);
        #endif

      }

    }
    else{
       state.FoodServe2 = LOW;
       #ifdef __DEBUG__CTRL_
        Serial.println(F("Reset FoodServe2 : LOW"));  //entering here
       #endif
    }


    //Serial.print( "Feed_Button"); Serial.println(digitalRead(Feed_Button));  
    if (digitalRead(Feed_Button) == LOW ) {     //active low
      
      #ifdef _DEBUG_Feed_Button_
          Serial.println( F("Feed_Button LOW")); 
      #endif

      state.FoodServe = HIGH;
      
    }
    
    if (state.FoodServe == HIGH) { //wrapping it in funtion does not work why??

            state.FilterCtrlPin = HIGH;
            digitalWriteFast(controlPinFilter, state.FilterCtrlPin); // high means filter pump is cut-off
                
            #ifdef _DEBUG_Feed_Button_
                Serial.println(" inside if (state.FoodServe == HIGH)");
            #endif
  
            servoFeed.attach(controlPinFoodServo);
            

            #ifdef __DEBUG__servo_  
                Serial.print("inside servoRun feedtimes : "); Serial.println(feedtimes);
            #endif

            if (feedserved < feedtimes ){

                //to make truely non blocking call oneself here with flag preventing entry to this block of code
              
                // for(servoPosition = servoMinDegrees; servoPosition < servoMaxDegrees; servoPosition+=2){                                  
                //   servoFeed.write(servoPosition);             
                //   //Serial.println(servoPosition); 
                //   lcd_option(LCD_Display);
                //   delay(15);                   
                // } 

                // for(servoPosition = servoMaxDegrees; servoPosition > servoMinDegrees; servoPosition-=2){                                
                //   servoFeed.write(servoPosition);    
                //   //Serial.println(servoPosition); 
                //   lcd_option(LCD_Display);       
                //   delay(15);       
                // }     

                bool servoDirection= HIGH;          // indicate optimized version of above
                servoPosition = servoMinDegrees;
                while (servoPosition >= servoMinDegrees){
                    servoFeed.write(servoPosition);
                    //Serial.print(state.servoDirection); Serial.print(" :");Serial.println(servoPosition); 
                    if (servoDirection){
                      servoPosition+=2;
                    }
                    else{
                      servoPosition-=2;
                    }
                    if (servoPosition >= servoMaxDegrees){
                      servoDirection= LOW;
                    }
                    delay(15);     
                    // #define servoMinDegrees 85  //the limits to servo movement
                    // #define servoMaxDegrees 130
                    // #define feedtimes 3
                    // 130-85=45
                    // 45/2*15ms ~ 340 ms < 500ms for the update of the lcd might miss just one beat in lcd
                  }
                
              feedserved++;              
              //Serial.print("feed Served :");Serial.println(feedserved);
              
              #ifdef __DEBUG__servo_  
                  Serial.print("feed Served :");Serial.println(feedserved);
              #endif    
            }
            else{
              state.FoodServe = LOW;
              feedserved=0;
            }

      if (state.FoodServe == LOW) {  // servo has finished its service
        millisFilterStartCommence = millisCurrent;
        #ifdef __DEBUG__feed_  
            Serial.print("millisFilterStartCommence :");Serial.println(millisFilterStartCommence);
        #endif        
      }
      servoFeed.detach();
    }


    #ifdef __DEBUG__CTRL_
      Serial.println(state.FoodServe);  //entering here
    #endif
  //feed_block

  //turn filter pump on after millisFilterPumpOffTime

  #ifdef __DEBUG__CTRL_
    Serial.print(F("Filter:"));  //entering here
  #endif


  if (state.FilterCtrlPin)  {//ie filter is off						// if filter pin is high toggle it on after delay of 30 min

    #ifdef __DEBUG__feed_
           Serial.print(F("Filter:"));  //entering here
    #endif

      if (millisCurrent - millisFilterStartCommence >= millisFilterPumpOffTime){
        state.FilterCtrlPin = LOW;  // low means filter pump on countdown to start filter pump started
        //start the filter pump again
        digitalWriteFast(controlPinFilter, state.FilterCtrlPin);
      }
  }
  #ifdef __DEBUG__CTRL_
    Serial.println(state.FilterCtrlPin); 
  #endif 

} //ControlState()


//========

void setup() {

  //state.FoodServe = HIGH;
  #ifdef __SERIAL.BEGIN__
      Serial.begin(57600);
      Serial.println( F("Starting SeveralThingsAtTheSameTimeAquarium") );
      //delay(100);
  #endif

  
  
  #ifdef __INTIAL_TIME_ESTIMATION_
    Serial.println(millisCurrent);
    Serial.println(__DATE__);
    Serial.println(__TIME__);
    Serial.println(F("IN THE LOOP"));
    Serial.println (  millis() - millisCurrent ); //print millis till here as millisCurrent is 0
  #endif

  
  servoFeed.attach(controlPinFoodServo);  //needed due to funny servo jitter while powering on which shifts the position to about 5  degree
  servoFeed.write(servoMinDegrees);       //may cause the feed to deliver unintentionally and all
   

  lcd.begin(col, row);
  lcd.clear();    
  lcdLnClrPos(0);     
  lcd.print(F("Aquarium"));
  lcdLnClrPos(1,4);
  lcd.print(F("Automation"));

  #ifdef __DEBUG__SETUP_
        Serial.println(  F("setup  here") );  //entering here
  #endif
  
  //RCT clock   //pin 20 sda and 21 scl
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
    // When time needs to be set on a new device, or after a power loss, this sets the RTC to the date & time this sketch was compiled
    // if compiled on January 21, 2014 at 3am you would call: rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }



  rtc_now = rtc.now();
  
      // temperature data from rtcClock
      // Serial.println( F("Temperature rtcClock: " + String(rtc.getTemperature()) + " °C") );  //test   slow heavy on memory


  #ifdef __DEBUG__SETUP_
        Serial.print( F("Temperature rtcClock:") );
        Serial.println(rtc.getTemperature());
        Serial.print(rtc_now.year(), DEC);
        Serial.print('/');
        Serial.print(rtc_now.month(), DEC);
        Serial.print('/');
        Serial.print(rtc_now.day(), DEC);
        Serial.print(' ');
        Serial.print(daysOfTheWeek[rtc_now.dayOfTheWeek()]);
        Serial.print(' ');
        Serial.print(rtc_now.hour(), DEC);
        Serial.print(':');
        Serial.print(rtc_now.minute(), DEC);
        Serial.print(':');
        Serial.print(rtc_now.second(), DEC);
        Serial.println();
  #endif

  //oneWire search Started

  #ifdef __DEBUG__TEMP_
        Serial.println( F("Dallas Temperature IC Control Library") );
        // locate devices on the bus
        Serial.println( F("Locating devices") );
  #endif

  oneWireSensors.begin();

  #ifdef __DEBUG__TEMP_
        Serial.print( F("Found") );
        Serial.print(oneWireSensors.getDeviceCount(), DEC);
        Serial.println( F(" devices") );

        // report parasite power requirements
        Serial.print( F("Parasite power is:") ); 
        if (oneWireSensors.isParasitePowerMode()){
      Serial.println(F("ON"));
          }
        else {
      Serial.println(F("OFF"));
        }
        //finding address of the Temperatur Sensor    // done just once as the connectioin are fixed
        if (!oneWireSensors.getAddress(insideThermometer, 0)) {
      Serial.println(F("ERROR ! Unable to find address for Device 0"));
          } 
        else {
      oneWireSensors.setResolution(9);
      Serial.print(F("Device 0 Address:"));
      printAddress(insideThermometer);
      Serial.println(F("Resolution set to 9 bit"));
        }
  #endif

	

  //ldc
    pinMode(contrastPin, OUTPUT);      //no effect for 1604 
    pinMode(backlightPin, OUTPUT);
    //lcd.begin(col, row);
    analogWrite(contrastPin, contrast_val);     //set some contrast
    analogWrite(backlightPin, backlight_val);   //set backlight on

  //create character;  //starts from  lcd.write(1);  _BUG_
    //character 0 is available to do animation
    // lcd.createChar(1, Char_centigrade);          //byte(0) Char_centigrade
    // lcd.createChar(2, Char_Lux);
    // lcd.createChar(3, Char_inputParameters();
      // break;PH);
    // lcd.createChar(4, Char_ArrowRight);         //byte(1) Char_ArrowRight
    // lcd.createChar(5, Char_F1);
    // lcd.createChar(6, Char_F2);

  //Rotory encoder and button interrupts
    pinModeFast (Rot_Encoder_CLK,INPUT);
    pinModeFast (Rot_Encoder_DT,INPUT);
    pinModeFast (Rot_Encoder_SW,INPUT_PULLUP);
    pinModeFast (Mode_Button, INPUT_PULLUP);    //low when pressed
    pinModeFast (Feed_Button, INPUT_PULLUP);    //low when pressed

    pinModeFast(currentPinLux ,INPUT);    
    pinModeFast(currentPin_pH ,INPUT);    


  //set out pin all digital 
    pinModeFast(controlPinHeater, OUTPUT);
    pinModeFast(controlPinFilter, OUTPUT);
    pinModeFast(controlPinLighting, OUTPUT);
    pinModeFast(controlPinAlarm, OUTPUT);
    pinModeFast(controlPinAlarmLED,OUTPUT);
    pinModeFast(onBoardLedPin, OUTPUT);
    //pinModeFast(indicatorLedPin, OUTPUT);

    // done inside when needed to remove servo noise servo run loop
    //servoFeed.attach(controlPinFoodServo);
    //servoFeed.write(servoPosition);										//sets the initial position
	

  //set initial environment
    digitalWriteFast(controlPinHeater, LOW);				//initial state of Heater pin heater is on
    digitalWriteFast(controlPinFilter, LOW);		//initial state of Filter pin Filter is on
    digitalWriteFast(controlPinLighting, LOW);
    digitalWriteFast(controlPinAlarm, LOW);
    digitalWriteFast(controlPinAlarmLED,LOW);
    digitalWriteFast(onBoardLedPin, LOW);						//initial state of LED pin  LED off
    //digitalWriteFast(indicatorLedPin);

  //retrive eeprom 

  													//retrive information saved in EEPRO
    
    delay(250);
    servoFeed.detach();  //this gives sufficient time to move more than 5 degrees towards the servoMinPosition cancelling the effect of poweron jitter
    
    retriveFromEEPROM();	
    
} //setup()


void loop() {
  #ifdef __DEBUG__LOOP_
    Serial.println(F("IN THE LOOP"));
  #endif


  //calls for action
  millisCurrent = millis();  //note internal clock in millisecond


  #ifdef __LOOP_TIME_ESTIMATION_
    Serial.print(millisCurrent - millisEndofLoop);
    Serial.println(F("ms IN THE LOOP"));
  #endif

  rtc_now = rtc.now(); 
  time_hh  = rtc_now.hour();
  time_min = rtc_now.minute();

  checkFeedTime = time_hh * 60 + time_min;

  #ifdef  __DEBUG__feed_
        Serial.print("rtc clock :"); Serial.print(time_hh); Serial.println(time_min);
        Serial.print("checkFeedTime  :"); Serial.println(checkFeedTime); 
        delay(100);
  #endif

  // not needed as void retriveFromEEPROM() can handle these condition in the else block
  //   if (x.flag != 1) {  //invalid data go to setup EEPROM  only first run case it should trigger this
  //     alarm(HIGH);
  //     Serial.println(F("DO initial setup"));
  //     lcdmenu(setupEEPROM);  //It is blocking until setup is done
  //     alarm(LOW);
  //   }
  //


  ReadCurrents();

  ControlState();

  //Blink(interval);  moved into control structure for off when low light

  lcdmenu(LCD_Display);

  if (MB_Event()) {
    lcd.clear();
    lcd.setCursor(1, 2);
    lcd.println(F("MB_Event"));
    delay(200);


    #ifdef __DEBUG__LOOP_
        Serial.println(F("Setup mode MB_Event Happened"));
    #endif

    lcdmenu(Update_EEPROM);  //loop stopped while reading ie set button is kept push

  }  


  #ifdef __DEBUG__LOOP_
    delay(1000);
  #endif

  #ifdef __LOOP_TIME_ESTIMATION_
    millisEndofLoop = millisCurrent;
  #endif

}  //loop()

/*
void lcdLnClrPos(uint8_t lineNo, uint8_t pos = 0) {  //nonblocking
  if (lineNo < 2) {
    lcd.setCursor(pos, lineNo);
  } else {
    lcd.setCursor(pos-4, lineNo);  //cause i got the funny LCD
  }
  //     0123456789012345
  lcd.print(F("                "));

  //lineNo<2?lcd.setCursor (pos,lineNo):lcd.setCursor (pos-4,lineNo);

  if (lineNo < 2) {
    lcd.setCursor(pos, lineNo);
  } else {
    lcd.setCursor(pos-4, lineNo);  //cause i got the funny LCD
  }
} //lcdLnClrPos
*/
void lcdLnClrPos(uint8_t lineNo, uint8_t pos = 0) {  //nonblocking
  if (lineNo >= 2) {
    pos-=4;
  } 
  lcd.setCursor(pos, lineNo);
  //     0123456789012345
  lcd.print(F("                "));
  lcd.setCursor(pos, lineNo);
} //lcdLnClrPos

void lcdmenu(lcd_option option) {  //stable works do not tweak further  nonblocking except when entering setup

  time_hh  = rtc_now.hour();
  time_min = rtc_now.minute();
  uint16_t temTime;

  if (lcdOptOld_val == option) {


    #ifdef __DEBUG__LCDMENU_
        Serial.println(F("lcd currentOption == oldOption"));
        Serial.println(millisCurrent);
        Serial.println(millisLastLCD);
    #endif

    if (millisCurrent - millisLastLCD < interval) {  // working


      #ifdef __DEBUG__LCDMENU_
            Serial.println(F("LCD not updated"));
      #endif


      return;

    } else {
      
      //Serial.println(lcd_recover);
      lcd.clear();
      if (!(lcd_recover%120)){    //reset lcd every 1 min 120*500ms=1s
           lcd.begin(col, row);   //to prevent the grabelling after glitch after with lcd is in byte mode not in nibble as set up by LiquidCrystal lcd(rs, en, d4, d5, d6, d7); 
           lcd.createChar(1, Char_centigrade);          //byte(0) Char_centigrade
           lcd.createChar(2, Char_Lux);
           lcd.createChar(3, Char_PH);
           lcd.createChar(4, Char_ArrowRight);         //byte(1) Char_ArrowRight
           lcd.createChar(5, Char_F1);
           lcd.createChar(6, Char_F2);
           lcd_recover = 0;     
      }
      lcd_recover++;      
      switch(k++){   //little animation
          case  0: 
          case  2:
            break;
          case 1:  
            lcd.createChar(7, Char_AA[0]);  
            break; 
          case 3:       
            lcd.createChar(7, Char_AA[1]);
            k=0;
            break;
      }     
      
      
      #ifdef __DEBUG__LCDMENU_
            Serial.println(F("LCD updated"));
      #endif
    }

  }

  switch (option) {

    case LCD_Demo:


        #ifdef  __DEBUG_LCD_DEMO_
            lcd.clear();
            lcd.print(F("Contrast:"));
            // loop through values for contrast
            for (int i = 0; i < 150; i++) {
              analogWrite(contrastPin, i);
              lcd.setCursor(0, 1);
              lcd.print(i);
              delay(80);
            }

            analogWrite(contrastPin, 50);
            lcd.clear();
            lcd.print(F("Back light:"));
            // loop through values for backlight
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
  
      lcdLnClrPos(1);
      lcd.print(F("Using Defaults"));     
      delay(1000);
      // inputParameters();
      // break;


    case Update_EEPROM:
      /*  //cant be seen anyway
      lcd.clear();
      lcdLnClrPos(0, 2);

      //          0123456789012345
      lcd.print(F("settingUp"));
      lcdLnClrPos(2, 4);
      lcd.print(F("input please"));
      delay(200);
      */

      inputParameters();
      break;


    case LCD_Display:  // update tweak for layout formating when all is finished


      lcdLnClrPos(0);
               // 012345
      lcd.print(F("Now  "));
      if (time_hh < 10) {
        lcd.print(F("0"));
      }
                 //67
      lcd.print(time_hh);
                 //8
      if (lcd_recover%2){
         lcd.print(":");
      }
      else{
         lcd.print(" ");
      }

      if (time_min < 10) {
        lcd.print(F("0"));
      }
                  //90
      lcd.print(time_min);
                 //12345
      lcd.print(F("  Set"));


      #ifdef __DEBUG__LCDMENU_
            Serial.println(F("rtc Clock"));
            if (time_hh < 10) {
              Serial.print(F("0"));
            }
            Serial.print(time_hh);
            Serial.print(F(":"));
            if (time_min < 10) {
              Serial.print(F("0"));
            }
            Serial.println(time_min);
      #endif


      lcdLnClrPos(1);
      //0
      lcd.write(1);  //degree centi
                     //12
      //lcd.print( F(":") );

      if (currentTemp < 100) {
        lcd.print(' ');
      }
      //12345
      lcd.print(currentTemp / 10.0, 1);

      lcd.setCursor(6, 1);  //possible bug here
      lcd.write(7);      
      lcd.write(7);     
      lcd.write(7);   
      lcd.write(7);
      lcd.write(7);

      lcd.setCursor(12, 1);
      
      lcd.print(x.setTemp / 10.0, 1);

      lcdLnClrPos(2, 0);
      // 0
      lcd.write(2);  //lux
                     // 1
      //lcd.print( F(":") );
               //2345678
      lcd.print(currentLight);

      //11
      lcd.setCursor(2, 2);   //possible bug here
      lcd.write(7);      
      lcd.write(7);     
      lcd.write(7); 
      lcd.write(7); 
      lcd.write(7);         
      
      
      lcd.setCursor(8, 2);
      //2345
      lcd.print(x.setLux);

      lcdLnClrPos(3, 0);
      // 0
      lcd.write(3);  //ph

      lcd.print(current_pH / 10.0, 1);

      lcd.setCursor(1, 3);

      lcd.write(5);  //F1

      temTime = x.feedTime1 / 60;

      if (temTime < 10) {
        lcd.print(0);
      }

      lcd.print(temTime);

      temTime = x.feedTime1 % 60;

      if (temTime < 10) {
        lcd.print(0);
      }

      lcd.print(temTime);

      lcd.setCursor(7, 3);

      lcd.write(6);  //F2

      temTime = x.feedTime2 / 60;

      if (temTime < 10) {
        lcd.print(0);
      }

      lcd.print(temTime);

      temTime = x.feedTime2 % 60;

      if (temTime < 10) {
        lcd.print(0);
      }

      lcd.print(temTime);


      #ifdef __DEBUG__LCDMENU_
            Serial.print(F("Temperature of rtc:"));
            Serial.print(rtc.getTemperature());
      #endif


      break;


    default:

      #ifdef __DEBUG__LCDMENU_
            Serial.print(F("Invalid option pased to lcdmenu"));  // statements for default value
      #endif

      break;
  }

  lcdOptOld_val = option;
  millisLastLCD = millisCurrent;

} //lcdmenu() // need lcd update code from here to the input parameters


void f_setTemp() {  //blocking


  #ifdef __DEBUG__f_setTemp_
    Serial.print(F("IN f_setTemp"));
  #endif

  unsigned long millisLastParameters = millis();
  uint8_t temp = x.setTemp;  //too detect the change
  lcd.clear();
  if (x.setTemp < 190 || x.setTemp > 325) {  //should never execute except some fault
    x.setTemp = 280;
    #ifdef __DEBUG__f_inputParameters_
            Serial.print(F("Invalid Setting to default 28°"));
    #endif
    saveToEEPROM();
  }
  while (1) {  // infinite loop blocking
               //    0123456789012345
    lcdLnClrPos(1);
    lcd.print(F("Temp Setting"));
    lcdLnClrPos(2, 4);
    lcd.print(x.setTemp / 10.0, 1);
    lcd.write(1);  //lcd line 2 and 3 puts tabs at intial so must be select -ve column

    switch (Rot_Encoder_Poll()) {


      case NO_CHANGE:
          if (millis() - millisLastParameters > 2000){  //time out 2 sec
          #ifdef __DEBUG__f_inputParameters_
            Serial.print(F("Retriving from EEPROM:"));
          #endif
          retriveFromEEPROM();
          return;                                  //read  variables from eeprom if 0 to reverse the changes introduced  //reverse change not implemnted yet
        }


      case CLOCKWISE:
        if (x.setTemp < 325)
          x.setTemp += 5;

        millisLastParameters = millis();
        break;


      case ANTICLOCKWISE:
        if (x.setTemp > 190)
          x.setTemp -= 5;

        millisLastParameters = millis();
        break;


      case SW_BUTTON:
        //save  variables to eeprom if 1 to record the changes introduced
        #ifdef __DEBUG__f_inputParameters_
            Serial.print(F("Saving to EEPROM:"));
        #endif

        saveToEEPROM();
        return;

        
    }

  }
} //f_setTemp()


void f_setLux() {  //blocking


  #ifdef __DEBUG__f_setLux_
    Serial.print(F("IN f_setLux"));
  #endif

  unsigned long millisLastParameters = millis();
  lcd.clear();
  while (1) {  // infinite loop blocking
               //    0123456789012345
    lcdLnClrPos(1);
    lcd.print(F("Light Setting"));
    lcdLnClrPos(2, 4);
    lcd.print(x.setLux);

    switch (Rot_Encoder_Poll()) {


      case NO_CHANGE:
        if (millis() - millisLastParameters > 2000){  //time out 2 sec
          #ifdef __DEBUG__f_inputParameters_
            Serial.print(F("Retriving from EEPROM:"));
          #endif
          retriveFromEEPROM();
          return;                                  //read  variables from eeprom if 0 to reverse the changes introduced  //reverse change not implemnted yet
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
        //save  variables to eeprom if 1 to record the changes introduced
        #ifdef __DEBUG__f_inputParameters_
            Serial.print(F("Saving to EEPROM:"));
        #endif

        saveToEEPROM();
        return;  
    }
  }
} //f_setLux() 


void f_setTime(byte sel) {

  String MenuTime_message[2] = {
  //    0123456789012
  "Set hh",
  "Set mm",
  };


  #ifdef __DEBUG__f_setTime_
    Serial.print(F("IN f_setTime"));
  #endif

  unsigned long millisLastParameters = millis();

  lcd.clear();
  byte selector = 0;  //0 for hh 1 for min
  String str[3] = { "Clock", "FeedTime1", "FeedTime2" };

  #ifdef __DEBUG__f_setTime_
      if (sel > 2) {
        Serial.println(F("invalid sel value passed in f_setTime function"));
        return ;
      }
  #endif


  switch (sel) {  //prelude
    case 0:


      #ifdef __DEBUG__f_setTime_
            Serial.print(F("sel"));
            Serial.print(str[sel]);
      #endif

      rtc_now = rtc.now();
      time_hh= rtc_now.hour();
      time_min= rtc_now.minute();
      break;

    case 1:


      #ifdef __DEBUG__f_setTime_
            Serial.print(F("sel"));
            Serial.print(str[sel]);
      #endif

      time_hh= x.feedTime1 / 60;
      time_min= x.feedTime1 % 60;
      break;

    case 2:

      #ifdef __DEBUG__f_setTime_
            Serial.print(F("sel"));
            Serial.print(str[sel]);
      #endif

      time_hh= x.feedTime2 / 60;
      time_min= x.feedTime2 % 60;
      break;

    default:


      #ifdef __DEBUG__f_setTime_feedTime
            Serial.println(F("out of selector loop sel == default"));
      #endif
      break;

  }  //end of the prelude to the setting times


  while (1) {  //interlude


    //setting hours

    #ifdef __DEBUG__f_setTime_
        Serial.println(F("in the fn f_setTime while 1 loop"));  // infinite loop blocking
    #endif


    lcdLnClrPos(0, 5);
    lcd.print(str[sel]);
    while (selector == 0) {


        #ifdef __DEBUG__f_setTime_
              Serial.println(F("in the fn f_setTime while selector == 0 loop"));
        #endif

        //                0123456789012345
        lcdLnClrPos(1);
        lcd.print(F("set HH (24)"));
        lcdLnClrPos(2, 6);
        if (time_hh < 10) {
        lcd.print(F("0"));
        }
        lcd.print(time_hh);


        #ifdef __DEBUG__f_setTime_
              Serial.println(F("in the fn f_setTime selector==0 loop"));
        #endif

      switch (Rot_Encoder_Poll()) {


        case NO_CHANGE:
          if (millis() - millisLastParameters > 2000) {  //time out 2 sec


            #ifdef __DEBUG__f_setTime_
                        Serial.println(F("selector==0           case NO_CHANGE:"));
            #endif

            return ;
          }  //read  variables from eeprom if 0 to reverse the changes introduced
          break;


        case CLOCKWISE:


          #ifdef __DEBUG__f_setTime_
                    Serial.println(F("selector==0           case case CLOCKWISE:"));
          #endif


          time_hh++;
          if (time_hh > 23) {
            time_hh= 0;
          }
          millisLastParameters = millis();
          break;


        case ANTICLOCKWISE:


          #ifdef __DEBUG__f_setTime_
                    Serial.println(F("selector==0           case case ANTICLOCKWISE:"));
          #endif


          if (time_hh > 0) {
            time_hh--;
          } else {
            time_hh= 23;
          }
          millisLastParameters = millis();
          break;


        case SW_BUTTON:


          #ifdef __DEBUG__f_setTime_
                    Serial.println(F("selector==0           case case SW_BUTTON:"));
          #endif

          selector = 1;  //save  variables to eeprom if 1 to record the changes introduced
      }

    }


    millisLastParameters = millis();


    #ifdef __DEBUG__f_setTime_
        Serial.print(F("before while selector == 1 loop  :"));
        Serial.println(selector);
    #endif



    //setting minutes

    while (selector == 1) {


      #ifdef __DEBUG__f_setTime_
            Serial.println(F("selector == 1 loop"));
            Serial.print(F("in while selector == 1 loop  :"));
            Serial.println(selector);
      #endif

      //   0123456789012345
      lcdLnClrPos(1, 4);
      lcd.print(F("set mm"));
      lcdLnClrPos(2, 6);

      if (time_min < 10) {
        lcd.print(F("0"));
      }
      lcd.print(time_min);


      //entry here

      switch (Rot_Encoder_Poll()) {



        case NO_CHANGE:
          if (millis() - millisLastParameters > 2000) {  //time out 2 sec


          #ifdef __DEBUG__f_setTime_
                      Serial.println(F("selector==1           case NO_CHANGE:"));
          #endif


            return ;
          }
          //read  variables from eeprom if 0 to reverse the changes introduced
          break;



        case CLOCKWISE:


            #ifdef __DEBUG__f_setTime_
                      Serial.println(F("selector==1           case CLOCKWISE:"));
            #endif

            time_min++;
          if (time_min > 59) {
            time_min= 0;
          }
          millisLastParameters = millis();
          break;



        case ANTICLOCKWISE:


          #ifdef __DEBUG__f_setTime_
                    Serial.println(F("selector==1           case ANTICLOCKWISE:"));
          #endif

          if (time_min > 0) {
            time_min--;
          } else {
            time_min= 59;
          }
          millisLastParameters = millis();
          break;



        case SW_BUTTON:


          #ifdef __DEBUG__f_setTime_
                    Serial.println(F("selector==1           case SW_BUTTON:"));
          #endif

          selector = 2;
          millisLastParameters = millis();
          break;  //save  variables to eeprom if 1 to record the changes introduced
      }
    }


    if (selector = 2) {
      //Serial.println(F("out of selector loop selector == 2"));


      #ifdef __DEBUG__f_setTime_
            Serial.println(F("code to update clock / feedtime1 /feedtime2"));
      #endif


      //  postlude
      switch (sel) {

          //set rtc clock
        case 0:


          #ifdef __DEBUG__f_setTime_
                    Serial.println(F("now.setTime(time_hh, time_min, 0);   setting the time"));
          #endif

          //uint32_t deltaT = 3600*hh+60*mm; // Move time ahead one hour
          rtc_now = rtc.now();
          //rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));


          #ifdef __DEBUG__f_setTime_setClock_
                    Serial.print(F("Before Adjust  :"));
                    Serial.print(rtc_now.year(), DEC);
                    Serial.print('/');
                    Serial.print(rtc_now.month(), DEC);
                    Serial.print('/');
                    Serial.print(rtc_now.day(), DEC);
                    Serial.print(' ');
                    Serial.print(daysOfTheWeek[rtc_now.dayOfTheWeek()]);
                    Serial.print(' ');
                    Serial.print(rtc_now.hour(), DEC);
                    Serial.print(':');
                    Serial.print(rtc_now.minute(), DEC);
                    Serial.print(':');
                    Serial.println(rtc_now.second(), DEC);
          #endif

          rtc.adjust(DateTime(rtc_now.year(), rtc_now.month(), rtc_now.day(), time_hh, time_min, 30));  //adjusting time working 


          #ifdef __DEBUG__f_setTime_setClock_
                    Serial.print(F("clock adjusted :"));
                    Serial.print(rtc_now.year(), DEC);
                    Serial.print('/');
                    Serial.print(rtc_now.month(), DEC);
                    Serial.print('/');
                    Serial.print(rtc_now.day(), DEC);
                    Serial.print(' ');
                    Serial.print(daysOfTheWeek[rtc_now.dayOfTheWeek()]);
                    Serial.print(' ');
                    Serial.print(rtc_now.hour(), DEC);
                    Serial.print(':');
                    Serial.print(rtc_now.minute(), DEC);
                    Serial.print(':');
                    Serial.print(rtc_now.second(), DEC);
          #endif

                
          rtc_now = rtc.now();
          return ;   //no need to update rom
          break;


          //set feedTime1

        case 1:
          // set time in some variable
          x.feedTime1 = time_hh * 60 + time_min;
          state.FoodServe1 = LOW;
                   

          #ifdef __DEBUG__f_setTime_feedTime
                    Serial.print(F("FeedTime1 set to"));
                    Serial.print(time_hh);
                    Serial.print(F(":"));
                    Serial.println(time_min);
          #endif

          saveToEEPROM(); // changes saved feedtime1 updated
          return ;  
          break;


          //set feedTime2

        case 2:
          x.feedTime2 = time_hh * 60 + time_min;
          state.FoodServe2 = LOW;


          #ifdef __DEBUG__f_setTime_feedTime
                    Serial.print(F("FeedTime2 set to"));
                    Serial.print(time_hh);
                    Serial.print(F(":"));
                    Serial.println(time_min);
          #endif
          saveToEEPROM(); //changes saved feedtime2 updated
          return ;  //no need to update eeprome
          break;

        default:


          #ifdef __DEBUG__f_setTime_feedTime
                    Serial.println(F("out of selector loop sel == default"));
          #endif

          
          return ;  
      }
    }


  }  
  //return 0;
} //f_setTime(byte sel)


void inputParameters() {  //blocking


  uint8_t menuTop = 0;  //for tracking the displayed menu
  uint8_t menuPosition = 0;
  uint8_t menuSubPosition = 0;


  const uint8_t NumMenu = 6;
  String MenuLvl1_message[NumMenu] = {
  // make sure the pointer  i has a limit set . currently all ram can be viewed by rotating below exit
  //    0123456789012
  "Set Temp",       //0
  "Set lux",        //1
  "Set Clock",      //2
  "Set FeedTime1",  //3
  "Set FeedTime2",  //4
  "Save/Exit"       //5
  };

  #ifdef __DEBUG__f_inputParameters_
    Serial.print(F("IN inputParameters"));
  #endif


  uint8_t whileFlag = 1;
  //temp
  //lux_limit
  //time clock
  //feedTime1
  //feedTime2
  //exit
  unsigned long millisLastInputParameters = 0;

  while (whileFlag) {

    millisLastInputParameters = millis();
    lcd.clear();


    #ifdef __DEBUG__f_inputParameters_
        Serial.print(F("menuposition  :"));
        Serial.println(menuPosition);
        Serial.print(F("menuTop       :"));
        Serial.println(menuTop);
    #endif

    if (menuPosition >= NumMenu)
      menuPosition--;

    for (uint8_t i = menuTop, j = 0; i < menuTop + 4; i++, j++) {  //mem smashing occuring here //fixed


    #ifdef __DEBUG__f_inputParameters_
          Serial.print(F("index  ->"));
          Serial.print(i);
          Serial.print(F(":"));
          Serial.println(MenuLvl1_message[i]);
          Serial.println(F("\n\n"));
    #endif

      lcdLnClrPos(j, 1);
      if (i == menuPosition) {
        lcd.write(4);
      }  //print right arrow
      else {
        lcd.print(' ');
      }
      lcd.print(MenuLvl1_message[i]);
    }


    // memo to self fixed here please do not change anything bruteforce bug fixing here
    switch (Rot_Encoder_Poll()) {  // unprermitted memory access problem posssibly interplay of menuposition and menutop

      case NO_CHANGE:
        if (millis() - millisLastInputParameters > 3000)  //

        #ifdef __DEBUG__f_inputParameters_
                  Serial.println(F("NO_CHANGE"));
        #endif

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
            //saveToEEPROM();
            whileFlag = 0;
            break;

          default:
          
            #ifdef __DEBUG__f_inputParameters_
                        Serial.println(F("some thing odd in the menuPosition inputParameters();"));  // for existing while loop
            #endif
            break;
        }

        break;
    }

  }

} //inputParameters()
