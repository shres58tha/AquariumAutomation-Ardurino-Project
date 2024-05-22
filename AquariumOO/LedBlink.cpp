#include "LedBlink.h"

void LED_Blink::init(uint8_t  pin, unsigned long ms = 500){
      LED_PIN = pin;
      blinkInterval = ms ;         // millis
      pinModeFast(LED_PIN, OUTPUT);
      digitalWriteFast(LED_PIN,0);  //off at start
}

void LED_Blink::on(){
      digitalWriteFast(LED_PIN,1);
}

void LED_Blink::off(){
      digitalWriteFast(LED_PIN,0);
}

void LED_Blink::toggle(){
      digitalToggleFast(LED_PIN);
}

bool LED_Blink::getState(){
      return digitalReadFast(LED_PIN);    // not sure about it think should use the state variable
}

void LED_Blink::blink(unsigned long millisCurrent ) {
      Serial.print(millisCurrent); 	Serial.println ("Blink Blink");
      if ( millisCurrent -  LastBlink >= blinkInterval) {
        digitalToggleFast(LED_PIN);							
        LastBlink += blinkInterval;					//increment the millis
      }
}
