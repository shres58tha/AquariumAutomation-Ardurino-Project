#include "pHMeter.h"

void pHMeter::init(uint8_t  pin){
    this->pH_PIN = pin;
    pinMode(pH_PIN, INPUT );
}

uint8_t pHMeter::getpH(){  // returns Lux*10
    //https://circuitdigest.com/microcontroller-projects/arduino-ph-meter
    float V0=0.0,pH, cal_val = 0.0;
    uint8_t n=3;  

    for(int i=0;i<n;i++){
    V0 += analogRead(pH_PIN);
    delay(10);
    }
    pH = -5.70 * V0*(5.0/1024)/n + 21.34 + cal_val;
    return (uint8_t) pH*10;
}

/*
void pHMeter::setTarget_pH(uint8 t){
      target_pH = t;
      return 1;
}*/
