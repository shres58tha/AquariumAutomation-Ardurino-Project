
#ifndef LDR_H
#define LDR_H
     #include <Arduino.h>
      #include <digitalWriteFast.h>    
      #include <stdint.h>
  //#define  currentPinLux A8													//use internal pullup resistor pin to read form ldr  valid                   
class LDR{
	private:
	    uint8_t LDR_PIN;
      uint16_t targetLux;
      //uint16_t inVal;
	public:
      LDR(){}     //null constructor  not needed as no other constructor created by default
      void init(uint8_t  pin);
      void getTargetVal();
      uint16_t get();
      bool setTargetLux(uint16_t t);
};
#endif /* LDR_H */
