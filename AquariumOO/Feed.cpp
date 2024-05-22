#include "Feed.h"

void Feed::init(uint8_t  pin, uint8_t p =90, uint8_t s = 5, uint8_t min=60 ,uint8_t max=120){
    FEED_PIN=pin;
    pos=p;
    swing =s;
    minDegrees=min; 
    maxDegrees=max;    
    pinModeFast(FEED_PIN, OUTPUT);     
    digitalWriteFast(FEED_PIN, LOW);
}

void Feed::begin(uint8_t  pin){
    servo.write(pos);										//sets the initial position
	  servo.attach(FEED_PIN);
}

void Feed::setPos(uint8_t p){
    this->pos=p;
}

void Feed::setSwing(uint8_t s){
    this->swing=s;

}

void Feed::setHowMany(uint8_t n){
  howMany=n;
}

uint8_t Feed::getPos(){
  return pos;
}
uint8_t Feed::getSwing(){
  return swing;
}

uint8_t Feed::getHowMany(){
  return howMany;
}

  

bool Feed::start(unsigned int long millisCurrent, uint8_t s=1) {
  if(state.FoodServe) {
    
		if ( millisCurrent -  lastFeed >= interval) {
			lastFeed += interval;						//its time for another move
			pos = pos + degreesStep;//degreesStep might be negative
			if ((pos >= servoMaxDegrees) || (pos <= servoMinDegrees)) {
 //if the servo is at either extreme change the sign of the degrees to make it move the other way
				degreesStep = - degreesStep;						//reverse direction
 //and update the position to ensure it is within range
				pos = pos + degreesStep;
			}
 //make the servo move to the next position
			myservo.write(pos);
 //and record the time when the move happened
		}
		if ( degreesStep <= servoMinDegrees) {				// servo retorned to initial position
			state.ServoSwitch = LOW;
		}
	}
}
    
  }



void Feed::set(bool toggle){         //Feed ser unset
  toggle=LOW;
  if (toggle) Serial.println ("Feed Blaring");
	digitalWriteFast(Feed_PIN , toggle) ;  
}
