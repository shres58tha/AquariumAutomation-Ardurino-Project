#ifndef CLOCK_H
#define CLOCK_H
    #include "RTClib.h"
    #define ALARMS_NUMBER 6
    // char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
     typedef struct AlarmClock{
              bool set :1;
              bool buff :3;
              uint16_t min :12;
          };

    class TimeRTC{
        private:
          RTC_DS3231 rtc;
          DateTime rcDateTime;
          uint16_t AL_min;  // alarm value in min
          AlarmClock Alarms[ALARMS_NUMBER+1]={0,0,0}; // alarm 0 indicate error  //set =0, buff=7
          uint16_t YY;
          uint8_t MM,DD,hh,mm;
        public:
          TimeRTC(){}; //null constructor
          bool init();
          DateTime cTime();
          uint16_t get_YYYY();
          uint8_t get_MM();
          uint8_t get_DD();
          uint8_t get_hh();
          uint8_t get_mm();
          uint8_t getAlarms();
          uint8_t getTemperature();
          AlarmClock getAlarm(uint8_t i);   //recall recursively for viewing and changing alarm
          void setClock(uint16_t YYYY, uint8_t MM, uint8_t DD, uint8_t hh, uint8_t mm ) ;
          bool setAlarm(uint8_t hh, uint8_t mm);
          bool delAlarm(uint8_t i);
          
    };


#endif /* CLOCK_H */
