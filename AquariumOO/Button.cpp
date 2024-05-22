#include "Button.h"


void PushButton::init(uint8_t  pin){
    this->BUTTON_PIN=pin;
    pinModeFast(BUTTON_PIN, INPUT_PULLUP);     // normal  high pushed low 
}


bool PushButton::poll(){ //Mode Button Event
      rstate.current=digitalRead(BUTTON_PIN);         //normal is the high
      if (rstate.last!=rstate.current) {          // detecting falling edge
      rstate.last=rstate.current;     
        if (!rstate.current){ 
          //if rstate.Currnt = 0 
          //Serial.print(" Falling edge of Mode_Button  :");
          return 1;
        }
        else 
          return 0;
      }
      else {
      rstate.last=rstate.current;  
      } 
      return 0;
}
