#ifndef ALARM_H
#define ALARM_H
      #include <Arduino.h>
      #include <digitalWriteFast.h>    
      #include <stdint.h>
  //#define  controlPinAlarm  36                      //for the alarm indicatro led and or buzzer
 
class Alarm{
	private:
	    uint8_t ALARM_PIN;
      uint8_t ALARM_LED;
      bool sound = 1;
	public:
  /*  for future work only
	union{  
      struct{
      bool last     :1;   //1 lsb  normal calues
      bool current  :1;   //1       bool SW_Current  :1;   //1
      bool toggle   :1;   //uint8_t counter  :6;   //00 + 0000 msb
      //uint8_t counter  :5;   //00 + 0000 msb
      };
      byte state = 0x0F; ;   //initilize all bitfields to 0  
    } rstate;
  */ 
  Alarm(){}     //null constructor  not needed as no other constructor created by default
  void init(uint8_t  AP, uint8_t ALARM_LED = 0);
	void set(bool ST);
  void mute();
  void unmute();
  
};

#endif /* ALARM_H */
