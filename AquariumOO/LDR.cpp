#include "LDR.h"

void LDR::init(uint8_t  pin){
   LDR_PIN=pin;
}
void LDR::getTargetVal( ){
    return targetLux;
}


uint16_t LDR::get(){  // returns Lux*10
      /*  Typically 
    RL=500/lux
    V0=5*(RL/(RL+R))
    V0=LDR_value*ADC_value   
    lux=((2500/V0)-500)/R
    https://emant.com/316002
    https://www.allaboutcircuits.com/projects/design-a-luxmeter-using-a-light-dependent-resistor/


    //to prevent short circuit when LDR is highly illluminated
        [GND]--R[10k ohm resistor]--| -- [LDR] -- [5V]
      |
        [Analog Input A0]
      Different equation here but better is to calibrate (curve fixing by correlation) taking mobile sensor as reference

      Condition	    Illumination(lux)
      Sunlight	    	107527
      Full Daylight	 	10752
      Overcast Day	 	1075
      Very Dark Day		107
      Twilight	    	10.8
      Deep Twilight		1.08
      Full Moon	    	0.108
      Quarter Moon  	0.0108
      Starlight	    	0.0011
      Overcast Night	0.0001
        */
      float V0=0.0,lux;    //ADC = 1/1024
      //pinMode(currentPinLux ,INPUT);    //make analog pin A8 as input
      uint8_t n=3;
      for( uint8_t i = 0; i<n; i++){// taking average of 10
        V0 += analogRead(A0);
        delay (10);   //wait 10 micros
      }
      lux=((2500/V0)*n-500)/10000;
      Serial.print("lux :");Serial.println(lux);
      return (int16_t) lux*10;  // ambient dim room   Well lit sunlighed room ~ 200 to 400  
}


bool LDR::setTargetLux(uint16_t t){
      targetLux = t;
      return 1;
}
