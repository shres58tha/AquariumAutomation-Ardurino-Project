#ifndef pHMETER_H
#define pHMETER_H
      #include <Arduino.h>
      #include <digitalWriteFast.h>    
      #include <stdint.h>
      //#define  currentPin_pH A9												//pin to read from pH meter // analog valid

class pHMeter{
	private:
	    uint8_t pH_PIN;
      uint8_t target_pH;
      //uint8_t temp;
	public:
      pHMeter(){}     //null constructor  not needed as no other constructor created by default
      void init(uint8_t  pH_PIN );
      uint8_t getpH();

};
#endif /* pHMeter_H */
