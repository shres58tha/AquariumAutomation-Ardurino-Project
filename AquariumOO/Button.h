#ifndef BUTTON_H
#define BUTTON_H


      #include <Arduino.h>
      #include <digitalWriteFast.h>    
      #include <stdint.h>

//    #define Rot_Encoder_CLK 28
//    #define Rot_Encoder_DT 29
//    #define Rot_Encoder_SW 30      //pressed when low
//    #define MODE_BUTTON 31         //low  when pressed
//   enum Rot_Encoder_Event {NO_CHANGE, CLOCKWISE, ANTICLOCKWISE, SW_BUTTON, TIME_OUT};
//    const char *const Rot_Encoder_Event_String[5]={"NO_CHANGE", "CLOCKWISE", "ANTICLOCKWISE", "SW_BUTTON", "TIME_OUT"};
    ///*Bit Fields Start Low in ardurino
   
class PushButton{
	private:
	    uint8_t BUTTON_PIN;
	public:
	union{  
      struct{
      bool last     :1;   //1 lsb  normal calues
      bool current  :1;   //1       bool SW_Current  :1;   //1
      //uint8_t counter  :6;   //00 + 0000 msb
      };
      byte state = 0x0F; ;   //initilize all bitfields to 0  
    } rstate;

  PushButton(){}     //null constructor
  //Button(uint8_t  bpin);   must use init in setup to initilize pins so no point in this constructor
  void init(uint8_t  pin);
	bool poll();
};
#endif /* BUTTON_H */
