#include <cstdint>
#include "Alarm.h"

void Alarm::init(uint8_t  ap, uint8_t al){
    this->ALARM_PIN=ap;
    this->ALARM_LED=al;
    pinModeFast(ALARM_PIN, OUTPUT);     
    digitalWriteFast(ALARM_LED, LOW);
    if(ALARM_LED){
          pinModeFast(ALARM_LED, OUTPUT);     
          digitalWriteFast(ALARM_LED, LOW);
    }

}


void Alarm::set(bool toggle){         //Alarm ser unset
  #ifndef __ALARM_MUTE_
  if (toggle) Serial.println ("Alarm Blaring");
    if (sound)
      digitalWriteFast(ALARM_PIN , toggle) ;  
  #endif
  if(ALARM_LED != 0)
  digitalWriteFast(ALARM_PIN , toggle) ; 
}

void Alarm::mute(){
  sound = 0;
}
void Alarm::unmute(){
  sound = 1;
}