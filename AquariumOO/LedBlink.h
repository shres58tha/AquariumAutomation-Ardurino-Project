#ifndef LEDBLINK_H
#define LEDBLINK_H
      /* Can be used for Alarm too Just use the blink */
      #include <Arduino.h>
      #include <digitalWriteFast.h>    
      #include <stdint.h>
      //#define  currentPinLux A8													//use internal pullup resistor pin to read form ldr  valid                   
      class LED_Blink{
        private:
            uint8_t LED_PIN;
            unsigned long blinkInterval;                     // millis
            unsigned long LastBlink;
            //bool state;
            
        public:
            LED_Blink(){}     //null constructor  not needed as no other constructor created by default
            void init(uint8_t  pin, unsigned long ms = 500);
            void on();
            void off();
            void toggle();
            void blink(unsigned long millisCurrent);
            bool getState();
      };

#endif /* LEDBLINK_H */
