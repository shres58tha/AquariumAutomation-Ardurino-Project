#include "RotaryEncoder.h"

//null constructor  allows object creation like Rot_Encoder RE1;
//Rot_Encoder_Event_String[5] {"NO_CHANGE", "CLOCKWISE", "ANTICLOCKWISE", "SW_BUTTON", "TIME_OUT"};
//parameterized constructor allows object creation like Rot_Encoder RE1= new Rot_Encoder(28,29,30,31,);
/*  //need to use init for pin initialization
Rot_Encoder::Rot_Encoder(uint8_t CLK,uint8_t DT,uint8_t SW){
    this->CLK=CLK;
    this->DT=DT;
    this->SW=SW; 
}*/

void Rot_Encoder::init(uint8_t CLK,uint8_t DT,uint8_t SW){
	  this->CLK=CLK;
    this->DT=DT;
    this->SW=SW;
    pinModeFast (CLK,INPUT);
    pinModeFast (DT,INPUT);
    pinModeFast (SW,INPUT_PULLUP);   //low when pressed    
}


Rot_Encoder_Event Rot_Encoder::poll(){ 
	  unsigned long millisLocal_SW; 
	  unsigned long millisTimeout=4000;
	  unsigned long millisInitial=millis();
	  delay(50);//debouncing possibly
		while(1){
		  millisLocal_SW=millis();
		  this->rstate.current_CLK = digitalReadFast(CLK);          
		  if (rstate.current_CLK != rstate.last_CLK){        // edge
			rstate.last_CLK = rstate.current_CLK;
			if(!rstate.current_CLK){                                    //falling edge
          if (digitalReadFast(DT)) {                              // if high rotating anticlockwise
          rstate.counter ++;
          Serial.print("Position ");Serial.print(rstate.counter);Serial.print(" : ");
          return ANTICLOCKWISE;   
          } else {
          rstate.counter --;
          Serial.print("Position ");Serial.print(rstate.counter);Serial.print(" : ");
          return CLOCKWISE;
          }
        } 
		  }
		  rstate.SW_Current=digitalReadFast(SW);
		  if (rstate.SW_Last!=rstate.SW_Current) {               // detecting falling edge
			rstate.SW_Last=rstate.SW_Current;   
			if (!rstate.SW_Current){
			  Serial.print(" Falling edge of button  :");
			  //delay(100);   // debouncing here may be not needed here
			  return SW_BUTTON;
		  }
		  }
		if (millisLocal_SW - millisInitial >millisTimeout)   //return if no action for 100  effectively making it non blocking
		  return TIME_OUT;
		delay(5);
  } 
}

