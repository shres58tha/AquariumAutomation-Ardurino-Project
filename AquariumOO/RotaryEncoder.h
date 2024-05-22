#ifndef ROTARYENCODER_H
    #define ROTARYENCODER_H

    #include <Arduino.h>
    #include <digitalWriteFast.h>    
    #include <stdint.h>
//    #define Rot_Encoder_CLK 28
//    #define Rot_Encoder_DT 29
//    #define Rot_Encoder_SW 30      //pressed when low
//    #define MODE_BUTTON 31         //low  when pressed
    enum Rot_Encoder_Event {NO_CHANGE, CLOCKWISE, ANTICLOCKWISE, SW_BUTTON, TIME_OUT};
    const char *const Rot_Encoder_Event_String[5]={"NO_CHANGE", "CLOCKWISE", "ANTICLOCKWISE", "SW_BUTTON", "TIME_OUT"};
   
   
class Rot_Encoder{
	private:
	    uint8_t CLK;
	    uint8_t DT;
	    uint8_t SW;
	public:
  ///*Bit Fields Start Low in ardurino
	union{  
      struct{
      bool SW_Current  :1;    //1 lsb  normal calues
      bool SW_Last     :1;   //1
      bool current_CLK :1;   //0  
      bool last_CLK    :1;   //0
      uint8_t counter  :4;   //0000 msb    lower nibble  counter 0-13
      };
      byte state = 0x0F; ;   //initilize all bitfields to 0  
    } rstate;

  Rot_Encoder(){}     //null constructor

  //Rot_Encoder(uint8_t CLK,uint8_t DT,uint8_t SW);   //need to use init for pin initialization
  
  void init(uint8_t CLK,uint8_t DT,uint8_t SW);

	Rot_Encoder_Event poll();

};

#endif /* ROTARYENCODER_H */
