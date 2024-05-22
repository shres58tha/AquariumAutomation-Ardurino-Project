#ifndef FEED_H
#define FEED_H

      #include <Arduino.h>
      #include <digitalWriteFast.h>    
      #include <stdint.h>
      #include <Servo.h>
      #define  MinDegrees 70												//the limits to servo movement 
      #define  MaxDegrees 100
      //  #define  controlPinFoodServo 	37									//pwn may be needed servo motor turn pin. Normal low. introduce feed when high

class Feed{
	private: 
	    uint8_t FEED_PIN;
      uint8_t pos;
      uint8_t swing;
      uint8_t howMany;
      int8_t degreesStep;
      uint8_t  minDegrees;												//the limits to servo movement 
      uint8_t  maxDegrees;      
      Servo servo;
      unsigned long lastFeed;
      unsigned long interval;
	public:
      init(uint8_t  pin, uint8_t pos =90, uint8_t swing = 5, uint8_t minDegrees=70 ,uint8_t maxDegrees=100);
      void setPos(uint8_t p);
      void setSwing(uint8_t s);
      void setdegreesStep(int8_t step);
      void setMinDegrees(uint8_t min);
      void setMaxDegrees(uint8_t max);
      
      uint8_t getPos();
      uint8_t getSwing();
      int8_t  getdegreesStep();
      uint8_t getMinDegrees();
      uint8_t getMaxDegrees();

      bool Feed::start(unsigned int long millisCurrent, uint8_t s=0);
 
};

#endif /* FEED_H */
