#include "Clock.h"

bool TimeRTC::init(){
    
    #ifndef ESP8266
      while (!Serial); // wait for serial port to connect. Needed for native USB
    #endif

    if (! rtc.begin()) {
      Serial.println("Couldn't find RTC");
      Serial.flush();
      return 0;
    }

    if (!rtc.lostPower()) {
      Serial.println("RTC lost power, let's set the time!");
      // When time needs to be set on a new device, or after a power loss, the
      // following line sets the RTC to the date & time this sketch was compiled
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      // This line sets the RTC with an explicit date & time, for example to set
      // January 21, 2014 at 3am you would call:
      // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    }

    this->Alarms[0].buff=7;  //for error indication

    return 1;    
}


DateTime TimeRTC::cTime(){
    rcDateTime=rtc.now();
    return rcDateTime;  
}

uint16_t TimeRTC::get_YYYY(){
    this->cTime();
    return rcDateTime.year();
}

uint8_t TimeRTC::get_MM(){
    this->cTime();
    return rcDateTime.month();
}

uint8_t TimeRTC::get_DD(){
    this->cTime();
    return (rcDateTime.day());
}

uint8_t TimeRTC::get_hh(){
    this->cTime();
    return (rcDateTime.hour());
}

uint8_t TimeRTC::get_mm(){
    this->cTime();
    return (rcDateTime.minute());
}

uint8_t TimeRTC::getAlarms(){
    uint8_t num=0;
    for( uint8_t i=0; i< 6; i++){
        if (this->Alarms[i].set ==1){  
            num++;}    
    }
}

uint8_t TimeRTC::getTemperature(){
    return (rtc.getTemperature());
}

AlarmClock TimeRTC::getAlarm(uint8_t i){
    if (i<ALARMS_NUMBER){
        return (this->Alarms[i]);
    }
    return  Alarms[0]  ; //set =0, buff=7
}

void TimeRTC::setClock(uint16_t YYYY, uint8_t MM, uint8_t DD, uint8_t hh, uint8_t mm ){
    return rtc.adjust(DateTime(YYYY, MM, DD, hh, mm, 0));
}
/*
typedef struct AlarmClock{
              bool set :1;
              bool buff :3;    //= 7 means error
              uint16_t min :12;
          };
*/
bool TimeRTC::setAlarm(uint8_t hh, uint8_t mm){
  uint16_t temp = hh*60 + mm;
  if (temp < 1440){
    for ( uint8_t i=1; i<=ALARMS_NUMBER; i++){  //0th location indicate error; valid location 1 to ALARMS_NUMBER
      if (Alarms[i].set == 0){
         Alarms[i].min = temp;
         return 1;
      }
    }
  }
  return 0;  
}

bool TimeRTC::delAlarm(uint8_t i){
  if (i > 0 && i < ALARMS_NUMBER){
    Alarms[i].set=0;
    return 1;
    }
  return 0;
}


//USE DateTime alarm object for diffencess

/*
  // the pin that is connected to SQW
  #define CLOCK_INTERRUPT_PIN 2
  //don't need the 32K Pin, so disable it
  // set alarm 1, 2 flag to false (so alarm 1, 2 didn't happen so far)    
  // if not done, this easily leads to problems, as both register aren't reset on reboot/recompile  

  //in setup
    rtc.disable32K();
    rtc.clearAlarm(1);
    rtc.clearAlarm(2);

    // stop oscillating signals at SQW Pin
    // otherwise setAlarm1 will fail
    rtc.writeSqwPinMode(DS3231_OFF);
    // turn off alarm 2 (in case it isn't off already)
    // again, this isn't done at reboot, so a previously set alarm could easily go overlooked
    rtc.disableAlarm(2);

    // schedule an alarm 10 seconds in the future
    if(!rtc.setAlarm1(
            rtc.now() + TimeSpan(10),
            DS3231_A1_Second // this mode triggers the alarm when the seconds match. See Doxygen for other options
    )) {
        Serial.println("Error, alarm wasn't set!");
    }else {
        Serial.println("Alarm will happen in 10 seconds!");
    }
}

void time() {
    // print current time
    char date[10] = "hh:mm:ss";
    rtc.now().toString(date);
    Serial.print(date);

    // the stored alarm value + mode
    DateTime alarm1 = rtc.getAlarm1();
    Ds3231Alarm1Mode alarm1mode = rtc.getAlarm1Mode();
    char alarm1Date[12] = "DD hh:mm:ss";
    alarm1.toString(alarm1Date);
    Serial.print(" [Alarm1: ");
    Serial.print(alarm1Date);
    Serial.print(", Mode: ");
    switch (alarm1mode) {
      case DS3231_A1_PerSecond: Serial.print("PerSecond"); break;
      case DS3231_A1_Second: Serial.print("Second"); break;
      case DS3231_A1_Minute: Serial.print("Minute"); break;
      case DS3231_A1_Hour: Serial.print("Hour"); break;
      case DS3231_A1_Date: Serial.print("Date"); break;
      case DS3231_A1_Day: Serial.print("Day"); break;
    }

    // the value at SQW-Pin (because of pullup 1 means no alarm)
    Serial.print("] SQW: ");
    Serial.print(digitalRead(CLOCK_INTERRUPT_PIN));

    // whether a alarm fired
    Serial.print(" Fired: ");
    Serial.print(rtc.alarmFired(1));

    // Serial.print(" Alarm2: ");
    // Serial.println(rtc.alarmFired(2));
    // control register values (see https://datasheets.maximintegrated.com/en/ds/DS3231.pdf page 13)
    // Serial.print(" Control: 0b");
    // Serial.println(read_i2c_register(DS3231_ADDRESS, DS3231_CONTROL), BIN);

    // resetting SQW and alarm 1 flag
    // using setAlarm1, the next alarm could now be configurated
    if (rtc.alarmFired(1)) {
        rtc.clearAlarm(1);
        Serial.print(" - Alarm cleared");
    }
    Serial.println();

    delay(2000);
  }

 void onAlarm() {
    Serial.println("Alarm occured!");
}    
//*/
