#define abs(x) ((x)>0?(x):-(x))
#include <digitalWriteFast.h>
#include "Bundled.h"
  /*  pinModeFast (pinNum)
  d igitalWriteFast(pinNum, state) (sets or* clears pin/port faster)
  pinModeFast(pinNum, mode) (sets pin/port as input or output faster)
  digitalReadFast(pinNum)(reads the state of pin/port faster)
  digitalToggleFast(pinNum)(toggles the state of pin/port faster)
  */
#ifndef  MENU_H
  #define MENU_H
#include <LiquidCrystal.h>
  //declare the lcd 2004 20 * 4 pins declarations
  //pin name        1   2   3   4   5   6  7    8   9   10  11  12  13  14   15         16
  //signal          VSS VDD V0  RS  R/W E  DB0  DB1 DB2 DB3 DB4 DB5 DB6 DB7  LED+       LED-
  //value rt to lf  gnd  5v pot      5v     na  na  na  na                   10kReg5V   gnd
  const int rs = 27, en = 26, d4 = 25, d5 = 24, d6 = 23, d7 = 22;
  const int col=20, row=4;
  LiquidCrystal    lcd(rs, en, d4, d5, d6, d7);	  	// object creation
  //LiquidCrystal  lcd (27, 26, 25, 24, 23, 22);		// object creation
  #define contrastPin 12   //pwm capable pin //not necessary for 1604 display must for 2004 display
  #define backlightPin 11  //pwm capable pin
  uint8_t contrast_val=200;
  uint8_t backlight_val=75;

  enum lcd_option {DoNothing, LCD_Demo, setupEEPROM, Update_EEPROM, LCD_Display};
  lcd_option lcdOptOld_val = DoNothing;
  void lcdmenu(lcd_option option);        //declaration of lcdmenu()

#endif

///*rotary encoder with button + 1 button input + 1 button feed interrupt*/
  #define Rot_Encoder_CLK 28
  #define Rot_Encoder_DT 29
  #define Rot_Encoder_SW 30      //pressed when low
  #define Mode_Button 31         //use internal pull up register for const when pressed low
  #define Feed_Interrupt_Button 19         //pin 19 supports interrupt.

 Rot_Encoder rEnc;
 PushButton MB;
 Alarm buzzer;
 TimeRTC clk; 
 LDR ldrLux;
 pHMeter pH;


//sensor ingroup
  
  #define  currentPinLux A8													//use internal pullup resistor pin to read form ldr  valid  
  #define  currentPin_pH A9												//pin to read from pH meter // analog valid
  #define  ONE_WIRE_BUS 10												//pin to read for temp sensor  digital one wire pin
//rotary encoder done


  DateTime now;

// temp int rct clock //ph sensors // temp in ph sensor too
// Thermometer
#include <OneWire.h>
#include <DallasTemperature.h>
  //onewire object creation for thermometer
  OneWire  oneWire(ONE_WIRE_BUS); 
  DallasTemperature oneWireSensors(&oneWire);
  DeviceAddress insideThermometer;                   // array to hold address inside thermometer

#include <stdint.h>
/*EEPROM Example
    EEPROM.read() – Read a byte from EEPROM.
    EEPROM.write() – Write a byte to EEPROM.
    EEPROM.update() – Write a byte to EEPROM only if the current value is different to previous value.
    EEPROM.get() – Read any datatype from EEPROM (float or struct or any datatype).
    EEPROM.put() – Write any datatype to EEPROM (float or struct or any datatype)

  struct MyObject {
    float field1;
    byte field2;
    char name[10];
  };

  eeAddress += sizeof(float);
  EEPROM.put(eeAddress, customVar);
  Serial.print("Written custom data type!);
  MyObject customVar; //Variable to store custom object read from EEPROM.
  EEPROM.get(eeAddress, customVar);

  Serial.println("Read custom object from EEPROM: ");
  Serial.println(customVar.field1);
  Serial.println(customVar.field2);
  Serial.println(customVar.name);
  }
  */
#include <EEPROM.h>



void printAddress(DeviceAddress deviceAddress){
  for (uint8_t i = 0; i < 8; i++)  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
  delay(100);
}

uint8_t getCelsius(DeviceAddress deviceAddress){
  float tempC = oneWireSensors.getTempC(deviceAddress);
  if(tempC == DEVICE_DISCONNECTED_C)  {
    Serial.println("Error: Could not read temperature data");
    return 0;
  }
  else return(uint8_t) tempC*10;
}


//GLobal Variables
//servo limits
  #define  servoMinDegrees 70												//the limits to servo movement 
  #define  servoMaxDegrees 100

//Servo  constants
 Servo myservo;																		//create servo object to control a servo
 int servoPosition = servoMinDegrees;							//the intial angle of the servo - starting at 0.
 int servoDegrees = 5;														//amount servo moves at each step
 
struct{       //use bit fields 
	bool HeatercontrolPin = LOW;										//normal state is low  --> heater is on
	bool LightControlPin = LOW;
	bool FoodServe = LOW;													//normal state is low  --> feed off
	bool readFlag = LOW ;                          //1 indicate that the changes to be updated in the EEPROM
	bool ServoSwitch = LOW ;												//motor off
	bool FilterControlPin = LOW;										// normal state is low --> filter pump on
  //rot encoder variables 
  //bool SW_Current = LOW;
  //bool SW_Last = HIGH; 
  //bool Mode_Button_Current = LOW;
  //bool Mode_Button_Last = HIGH;                  //is high when button is not pressed
  //bool currentRot_Encoder_CLK=LOW;
  //bool lastRot_Encoder_CLK=LOW; 
} state;

//variables
union{
	struct{
		byte blank=0;																	//1 byte  0
		byte flag=0;																	//1 byte  1    valid data means flag 1
		uint16_t setTemp;														  //1 byte  2    in temp*10 scale
		uint16_t setLux;															//1 byte  5    int lux*10 scale
		unsigned long setFeedInterval;								//4 byte  7    millis set it if the rtc clock fails or use it with conjuction of rtc clock
		unsigned long setLightOnDuration;							//4 byte  11   millis set light on duration for option to turn off at night
	};																							//15 bytes
	byte byteData[20];															//for saving these into EEPROM // 2 forextra save space
} x ;

//variable here
  uint16_t currentTemp;															// use deci for both in val*10 scale  Range 10 unit
  uint16_t currentLux;                       				// uint16_t to accomodate 1024 levels the output of ADC  2 bytes
  uint8_t current_pH;															  // range 0 to 140 val*10
  uint8_t normal_pH_7=70;
  uint8_t range_pH= 10;                              //+-10 ie ph 6 to 8
//

//time variables
 unsigned long millisCurrent = 0;									//stores the value of millis() in each iteration of loop()
 unsigned long millisLastRead = 0;									//will store last time anlog input were read
 unsigned long millisLastCurrent = 0 ;

 unsigned long readInterval = 500 ;								//milliseconds
 unsigned long currentInterval = 500;							
 unsigned long servoInterval = 100 ;

 unsigned long millisLastServo = 0;								//the time when the servo was last moved

 unsigned long millisLastFed = 0;									//fed number of millis last
 unsigned long millisFoodInterval = 0;
 unsigned long millisFilterOffCommence = 0;				//for keeping track of filter pump off period
 unsigned long millisFilterPumpOffTime = 1800000;	//turn pump off for half hr in case food is delivered
 unsigned long millisDefaultBlink = 500;					//number of millisecs between blinks in millis
 unsigned long millisLastBoardLed = 0;            //last board blink in millis

 unsigned long millisLastLCD = 0;                 //update tracker for the LCD to avoid flicker
 unsigned long millisOverflow = 0 ;							  // not needed  can be used to extend millies counter to 64 bit


// custom characters
byte Char_centgrade[8] = {
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
byte Char_Food[8] = {
  B01000,
  B11100,
  B01010,
  B00111,
  B00010,
  B01000,
  B11100,
  B01001
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
// https://maxpromer.github.io/LCD-Character-Creator/


//alu  yaha cha hai
//output pin group
  #define  controlPinHeater 	32										// heater default state low  while high cutoff the heater
  #define  controlPinFilterPump 33									// default state low high cutoff
  #define  controlPinLighting 	34								//for controlling aquarium light normal is low.
  #define  controlPinUnknown 	35										// relay for future ..
  #define  controlPinAlarm  36                      //for the alarm indicatro led and or buzzer
  #define  controlPinFoodServo 	37									//pwn may be needed servo motor turn pin. Normal low. introduce feed when high
 
  #define  onBoardLedPin 13												//onBoard LED used to indicate operation state by blinking every x loops


void setFoodServeHigh(){
  state.FoodServe = HIGH;
	servoSweep();
}

void servoSweep() {
	if(state.FoodServe) {
		if ( millisCurrent -  millisLastServo >= servoInterval) {
			millisLastServo += servoInterval;						//its time for another move
			servoPosition = servoPosition + servoDegrees;//servoDegrees might be negative
			if ((servoPosition >= servoMaxDegrees) || (servoPosition <= servoMinDegrees)) {
 //if the servo is at either extreme change the sign of the degrees to make it move the other way
				servoDegrees = - servoDegrees;						//reverse direction
 //and update the position to ensure it is within range
				servoPosition = servoPosition + servoDegrees;
			}
 //make the servo move to the next position
			myservo.write(servoPosition);
 //and record the time when the move happened
		}
		if ( servoDegrees <= servoMinDegrees) {				// servo retorned to initial position
			state.ServoSwitch = LOW;
		}
	}
}

void Blink(unsigned long blinkDuration ) {
	Serial.print(millisCurrent); 	Serial.println ("Blink Blink");
  if ( millisCurrent -  millisLastBoardLed >= blinkDuration) {
		digitalToggleFast(onBoardLedPin);							//real Toggle herein
    //digitalToggleFast(indicatorLedPin);
		millisLastBoardLed += blinkDuration;					//increment the millis
	}
}

void retriveFromEEPROM() {
  Serial.println("Retriving from EEPROM");
	if( !( EEPROM.read(0) ) && EEPROM.read(1) ) {
		for( uint8_t i=0; i<sizeof(x); i++) {
			x.byteData [i]=EEPROM.read(i);
      Serial.println("");
      Serial.print("x.byteData");Serial.println(i);Serial.print(":");Serial.println(x.byteData[i]);
		}
	}
  //delay(100000);
}
/*
  customVar = (MyObject){ "sth", "next", 10298763, "#101#23"};
  int eeAddress = 0;
  EEPROM.put(eeAddress, customVar);
  */
void saveToEEPROM() {
	x.flag = 1;
	for( uint8_t i=0; i<=sizeof(x); i++) {
		EEPROM.update(i,x.byteData[i]);
	}
}

void ReadCurrents() {     //need updating
	//float tmpTemp = currentTemp , tmpLux = currentLux , tmp_pH = current_pH;
	if (millisCurrent - millisLastCurrent>= currentInterval) {
		currentTemp = getCelsius(insideThermometer) ;
		currentLux  = ldrLux.get()   ;     //ok  // normal scale
		current_pH  = pH.getpH()   ;     //ok  set ph into deci
		millisLastCurrent = millis() ;								//putting most recent value of millis in last read one.
	}
}

void ControlState() {
  //Serial.println( "control state control state control state control state control state control state control state control state");  //entering here
  //heater control
	if ( currentTemp > x.setTemp )
		state.HeatercontrolPin = HIGH;								//becomes high  heater off
	else
		state.HeatercontrolPin = LOW;									//becomes low   heater on
	digitalWriteFast(controlPinHeater, state.HeatercontrolPin);

  //Light control
	if ( currentLux < x.setLux  )
		state.LightControlPin = HIGH ;								//becomes high Light on
	else
		state.LightControlPin = LOW;									//becomes low lights off
	digitalWriteFast(controlPinLighting,LOW);

  //pH temperature range alarm
	if (  abs(current_pH - normal_pH_7) > 10 || abs (currentTemp - x.setTemp) > 20 ) {    //stored units are decis ie 20.0 stored as 200
		buzzer.set(HIGH);
	}
	else {
		buzzer.set(LOW);
	}

  //food control
	if ( state.FoodServe == LOW ) {									//pin low
		if ( ( millisCurrent - millisLastFed) > millisFoodInterval  ) {
			state.FoodServe =  HIGH;										//becomes high
			millisLastFed = millisCurrent + millisFoodInterval;
			state.FilterControlPin= HIGH;
			digitalWriteFast(controlPinFilterPump, state.FilterControlPin) ;//pin high here filter off
			servoSweep();
		}
	}
	if ( state.FoodServe == HIGH ) {
		servoSweep();
		if (state.FoodServe == LOW) {									// servo has finished its service
			millisFilterOffCommence = millisCurrent;
			state.FilterControlPin= LOW;								// low means filter pump on
		}
	}

  //turn filter pump after millisFilterPumpOffTime
	if ( !state.FilterControlPin )									// if filter pin is low turn it on after delay of 30 min in millisFoodInterval
		if ( millisCurrent - millisFilterOffCommence >= millisFoodInterval)
  //start the filter pump again
			digitalWriteFast(controlPinFilterPump, state.FilterControlPin) ;
}
//========
void setup() {
 
	Serial.begin(57600);
  
  rEnc.init( Rot_Encoder_CLK, Rot_Encoder_DT,Rot_Encoder_SW);   //pin 18 rx and 19 tx
  MB.init(Mode_Button);
  buzzer.init(controlPinAlarm);   
  //RCT clock
  if (!clk.init()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    delay(100);
  }
  ldrLux.init( currentPinLux );
  pH.init(currentPin_pH);
  

  // temperature data from rtcClock
  // Serial.println("Temperature rtcClock: " + String(rtc.getTemperature()) + " °C");  //test   slow heavy on memory
  Serial.println("Temperature rtcClock: " );
  Serial.print(clk.getTemperature());
  Serial.println( );

  //oneWire search Started
  Serial.println("Dallas Temperature IC Control Library");

  // locate devices on the bus
  Serial.print("Locating devices...");
  oneWireSensors.begin();
  Serial.print("Found ");
  Serial.print(oneWireSensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: "); 
  if (oneWireSensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");
  //finding address of the Temperatur Sensor    // done just once as the connectioin are fixed
  if (!oneWireSensors.getAddress(insideThermometer, 0)) { Serial.println(" ERROR ! Unable to find address for Device 0");} 
  else {oneWireSensors.setResolution(9);Serial.print("Device 0 Address: "); printAddress(insideThermometer);  Serial.println(" Resolution set to 9 bit");}

  //therm.init( ); //thermometer is using one wire bus change the class if it is shared not needed as initiated by onewire.h header and dallastemperature.h


  now = clk.cTime();


  




//
	Serial.println("Starting SeveralThingsAtTheSameTimeAquarium");
//ldc
  pinMode(contrastPin, OUTPUT);      //no effect for 1604 
  pinMode(backlightPin, OUTPUT);
  lcd.begin(col, row);
  analogWrite(contrastPin, contrast_val);     //set some contrast
  analogWrite(backlightPin, backlight_val);   //set backlight on

//create character;  //starts from  lcd.write(1);
  lcd.createChar(1, Char_centgrade);          //byte(0) Char_centgrade
  lcd.createChar(2, Char_Lux);
  lcd.createChar(3, Char_Food);
  lcd.createChar(4, Char_PH);
  lcd.createChar(5, Char_ArrowRight);         //byte(1) Char_ArrowRight

  attachInterrupt(digitalPinToInterrupt(Feed_Interrupt_Button), setFoodServeHigh, FALLING);     // ok will 100% work  

//set out pin all digital 
	pinModeFast(controlPinHeater, OUTPUT);
	pinModeFast(controlPinFilterPump , OUTPUT);
	pinModeFast(controlPinLighting, OUTPUT);
	//pinModeFast(controlPinFoodServo, OUTPUT);
	//pinModeFast(onBoardLedPin, OUTPUT);
  //pinModeFast(indicatorLedPin, OUTPUT);

	//myservo.write(servoPosition);										//sets the initial position
	//myservo.attach(controlPinFoodServo);

//set initial environment
	digitalWriteFast(controlPinHeater, LOW);				//initial state of Heater pin heater is on
	digitalWriteFast(controlPinFilterPump, LOW);		//initial state of Filter pin Filter is on
	digitalWriteFast(controlPinLighting, LOW);
	digitalWriteFast(controlPinAlarm, LOW);
	//digitalWriteFast(controlPinFoodServo, LOW);
	digitalWriteFast(onBoardLedPin, LOW);						//initial state of LED pin  LED off
  //digitalWriteFast(indicatorLedPin);

//retrive eeprom 
	retriveFromEEPROM();														//retrive information saved in EEPRO
}
void loop() {
  now = clk.cTime();
     //calls for action
	millisCurrent = millis();											   //note internal clock in millisecond
  if ( x.blank != 0 ){                             // found that it never actually executes expect when put in brandnew arduino all working
    if ( x.flag != 1 ){                            //invalid data go to setup EEPROM  only first run case it should trigger this
		buzzer.set(HIGH);
    Serial.println(" DO initial setup  ");
    lcdmenu(setupEEPROM);       //It is blocking until setup is done
    buzzer.set(LOW);
     }
	}
  /*  // this code works in in place but not in function
  state.Mode_Button_Current=digitalReadFast(Mode_Button);
	if (state.Mode_Button_Last && !state.Mode_Button_Current) { // detecting falling edge  true when state.Mode_Button_Last high state.Mode_Button_Current is low
		Serial.println(" Setup mode  ");
    lcdmenu(Update_EEPROM);												  //loop stopped while reading ie set button is kept pushe
	}
  state.Mode_Button_Last=state.Mode_Button_Current;
  */
  if (MB.poll()){
  	Serial.println(" Setup mode  ");
    lcdmenu(Update_EEPROM);												  //loop stopped while reading ie set button is kept pushe
  }
	ReadCurrents();

	ControlState();
  
	if (  abs(current_pH - normal_pH_7) > range_pH || abs (currentTemp - x.setTemp) > 10 ) {
		buzzer.set(HIGH);
	}
	else {
		buzzer.set(LOW);
	}

	Blink(millisDefaultBlink);

	lcdmenu(LCD_Display);

}

void lcdLnClr(uint8_t lineNo){
        //   0123456789012345
  lcd.setCursor (0,lineNo); lcd.print("                ");
}
void lcdmenu(lcd_option option){                 //stable works do not tweak further

  if (lcdOptOld_val == option){
    //Serial.println(" lcd currentOption == oldOption");
    //Serial.println(millisCurrent);
    //Serial.println(millisLastLCD);
    if (millisCurrent - millisLastLCD < 500){    // working
    Serial.println("LCD not updated");
    return;
    }
    else {
    Serial.println("LCD updated");
    }
  }

  switch (option) {
    case LCD_Demo:
      lcd.clear();
      lcd.print("Contrast:");
      // loop through values for contrast
      for (int i=0; i <= 150; i++){
        analogWrite(contrastPin, i);
        lcd.setCursor(0, 1);lcd.print(i);
        delay(80);
      }
      analogWrite(contrastPin, 50);
      lcd.clear();
      lcd.print("Back light:");
      // loop through values for backlight
      for (int i=0; i <= 255; i+=2){
        analogWrite(backlightPin, i);
        lcd.setCursor(0, 1); lcd.print(i);
        delay(80);
      }
      analogWrite(contrastPin, contrast_val);     //set some contrast
      analogWrite(backlightPin, backlight_val);   //set backlight on

      break;
   
    case setupEEPROM:  // this should be blocking 
      lcd.clear();
   //                                  0123456789012345
      lcd.setCursor (0,0);  lcd.print(" EEPROM Data ERR ");
      lcd.setCursor (0,1);  lcd.print(" DO INTIAL SETUP ");
      //lcd.setCursor (-4,2); lcd.print("                 ");
      //lcd.setCursor (-4,3); lcd.print("                 ");
      delay(1000);
      inputParameters();
      break;

    case Update_EEPROM:
      lcd.clear();
    //                                 0123456789012345
      lcd.setCursor (0,0);  lcd.print("   settingUp     ");
      //lcd.setCursor (0,1);  lcd.print("");
      lcd.setCursor (-4,2); lcd.print("  input please   ");
      //lcd.setCursor (-4,3); lcd.print("");
      delay(500);

      inputParameters();      
      break;

    case LCD_Display:     // this format is stable do not tweak any more
      //   lcd.clear();   
     
      
      lcdLnClr(0);                 // 0123                45                    6              78                       901             2345
      lcd.setCursor(0, 0); lcd.print("Now "); lcd.print(now.hour(),DEC); lcd.print(":"); lcd.print(now.minute(),DEC); lcd.print("  ");lcd.print(" Set  ");
      lcdLnClr(1);                 // 0             12                    34678                    90              12345
      lcd.setCursor(0, 1);  lcd.write(1);lcd.print(": ");lcd.print(currentTemp/10.0,1);lcd.print("  "); lcd.print(x.setTemp/10.0) ;
      lcdLnClr(2);                 // 0            1234              5678                901              2345
      lcd.setCursor(-4, 2); lcd.write(2);lcd.print(": ");lcd.print(currentLux);lcd.print("   "); lcd.print(x.setLux) ;
      lcdLnClr(3);                 // 0            123             456                           78            9            012              35
      lcd.setCursor(-4, 3); lcd.write(3);lcd.print(": ");lcd.print(x.setFeedInterval);lcd.print("  ");lcd.write(4);lcd.print(": ");lcd.print(current_pH);  
  
    Serial.print("Temperature of rtc: "); Serial.print(clk.getTemperature()); Serial.write(1);

        break;
      
      default:
        Serial.print("Invalid option pased to lcdmenu");// statements for default value
        break;
  }
  lcdOptOld_val = option;
  millisLastLCD = millisCurrent;  
}  // need lcd update code from here to the input parameters


uint8_t menuTop=0;              //for tracking the displayed menu
uint8_t menuPosition=0;
uint8_t menuSubPosition=0;

String MenuLvl1_message[5]= { 
  //    0123456789012
       "Set Temp.    ",  //0
       "Set lux      ",  //1
       "Set Clock    ",  //2
       "Set Time1st  ",  //3
       "Set FeedIntvl"   //4
};

String MenuTime_message[2]= { 
  //    0123456789012
       "Set hh",
       "Set mm",
};

bool f_setTemp(){
  unsigned long millisLastParameters=millis();
  lcd.clear();
  while(1){                    // infinite loop blocking
                               //    0123456789012345
      lcd.setCursor(0,1); lcd.print("  Temp Setting  ");
      lcd.setCursor(0,2); lcd.print(x.setTemp/10.0) ;lcd.write(1);     //lcd line 2 and 3 puts tabs at intial so must be select -ve column

    switch ( rEnc.poll() )   {
        case NO_CHANGE:
          if (millis()-millisLastParameters > 2000)    //time out 2 *sec
            return 0;                                  //read  variables from eeprom if 0 to reverse the changes introduced
          break;
        case CLOCKWISE:
          if (x.setTemp <320)
            x.setTemp++;
          millisLastParameters=millis();
          break;
        case ANTICLOCKWISE:
          if (x.setTemp >=210)
            x.setTemp--;
          millisLastParameters=millis();
          break;
        case SW_BUTTON:
          return 1;                                   //save  variables to eeprom if 1 to record the changes introduced
    }
  }
}

bool f_setLux(){
  unsigned long millisLastParameters=millis();
  lcd.clear();
  while(1){                    // infinite loop blocking
                               //    0123456789012345
      lcd.setCursor(0,1); lcd.print(" Light Setting  ");
      lcd.setCursor(0,2); lcd.print(x.setLux/10.0);

    switch ( rEnc.poll() )   {
        case NO_CHANGE:
          if (millis()-millisLastParameters > 2000)    //time out 2 sec
            return 0;                                  //read  variables from eeprom if 0 to reverse the changes introduced
          break;
        case CLOCKWISE:
          if (x.setLux <5000)
            x.setLux++;
          millisLastParameters=millis();
          break;
        case ANTICLOCKWISE:
          if (x.setLux >=1000)
            x.setLux--;
          millisLastParameters=millis();
          break;
        case SW_BUTTON:
          return 1;                                   //save  variables to eeprom if 1 to record the changes introduced
    }
  }
}
/*
String MenuLvl2Time_message[5]= { 
  //    0123456789012
       "Set Clock    ",  //0
       "Set Time1st  ",  //1
       "Set FeedIntvl"   //2
};*/
bool f_setTime(byte sel){
  unsigned long millisLastParameters=millis();
  byte hh=0, mm=0;  
  lcd.clear();
  if (sel > 2){
    Serial.println("invalid sel value passed in f_setTime function");
    return 0;   
  } 
  byte selector=0; //0 for hh 1 for min
  String str[3] = {"Clock","Time1st","FeedIntvl"} ;

  while(1){                    // infinite loop blocking
    lcd.setCursor(0,5); lcd.print(str[sel]);
    while(selector==0){
                               //    0123456789012345
      lcd.setCursor(0,1); lcd.print("    set hh");
      lcd.setCursor(0,2); lcd.print(hh) ;

      switch ( rEnc.poll() )   {
          case NO_CHANGE:
            if (millis()-millisLastParameters > 2000)    //time out 2 sec
              return 0;                                  //read  variables from eeprom if 0 to reverse the changes introduced
            break;
          case CLOCKWISE:
            if (hh <23)
              hh++;
            millisLastParameters=millis();
            break;
          case ANTICLOCKWISE:
            if (hh >0)
              hh--;
            millisLastParameters=millis();
            break;
          case SW_BUTTON:
            selector=1;                                   //save  variables to eeprom if 1 to record the changes introduced
      }
    }
    while(selector==1){
                               //   0123456789012345
      lcd.setCursor(0,1); lcd.print("  set mm  ");
      lcd.setCursor(0,2); lcd.print(mm) ;

      switch ( rEnc.poll() )   {
          case NO_CHANGE:
            if (millis()-millisLastParameters > 2000)    //time out 2 sec
              return 0;                                  //read  variables from eeprom if 0 to reverse the changes introduced
            break;
          case CLOCKWISE:
            if (mm <59)
              hh++;
            millisLastParameters=millis();
            break;
          case ANTICLOCKWISE:
            if (mm >0)
              hh--;
            millisLastParameters=millis();
            break;
          case SW_BUTTON:
            selector = 2;                                   //save  variables to eeprom if 1 to record the changes introduced
      }
      if (selector = 2){
        //code to update clock / starttime /interval

              switch(sel){
                case 0:
                  // now.setTime(hh, mm, 0);   //setting the time
                  break;
                case 1:
                  // set time in some variable
                  break;
                case 2:
                  //    set interval in some variable       
                  break;
                default:
                  return 1;    //flag to update EEPROM
              }   
           
      }      
    }

  }
}

void inputParameters(){
    //track entry
  state.readFlag = 0;
  uint8_t whileFlag=1;
  lcd.clear();
  //temp
  //lux_limit
  //time clock
  //start_feeding time set
  //feed_interval
  unsigned long millisLastInputParameters = 0;
  while(whileFlag){

    // need delay function here sad life
    //delay(50);

    //alu bako thau puchar yeha bata samau
    millisLastInputParameters=millis(); 
    lcd.clear();  
    for(uint8_t i = menuTop,j=0; i<menuTop+4; i++,j++)  {
      if ( j<2) lcd.setCursor(2,j);
      else      lcd.setCursor(-4,j);
      if( i== menuPosition) lcd.write(5);   //print right arrow
      else lcd.print(" ");
      lcd.print(MenuLvl1_message[i]);  
    }

    switch ( rEnc.poll() )   {
        case NO_CHANGE:
          if (millis()-millisLastInputParameters > 5000)  //
            whileFlag=0;
          break;
        case CLOCKWISE:
          if (menuPosition < 5)                       // increment menu position to max items  total of items 5
            menuPosition++;
          if (menuPosition >=4 && menuPosition < 5)   //scrolls
            menuTop++;    
          millisLastInputParameters=millis();     // reset timeout
          break;
        case ANTICLOCKWISE:
          if (menuPosition > 0)
            menuPosition--;
          if (menuPosition >=4  && menuTop > 0) 
            menuTop--;   
          millisLastInputParameters=millis();    // reset timeout
          break;
        case SW_BUTTON:
                  switch(menuPosition){
                    whileFlag=0;
                    case 0:
                      state.readFlag=f_setTemp();
                      break;  
                    case 1:
                      state.readFlag=f_setLux();
                      break; 
                    case 2:
                      state.readFlag=f_setTime(0);
                      break;  
                    case 3:
                      state.readFlag=f_setTime(1);
                      break;  
                    case 4:
                      state.readFlag=f_setTime(2);
                      break;
                    default:
                      Serial.println(" some thing odd in the menuPosition inputParameters();");   // for existing while loop            
                  }
          break;
    }
  }
  
	if( state.readFlag) {
		saveToEEPROM();
		state.readFlag = 0;
	}
}

