#include "fireSensorArray.h"

FireSensorArray::FireSensorArray(){
	for(int i=0; i<NUMFIRESENSORS; i++){
		fireSensePins[i] = -1;
	}
}

void FireSensorArray::attach(int pins[NUMFIRESENSORS]){
	for(int i=0; i<NUMFIRESENSORS; i++){
		fireSensePins[i] = pins[i];
		pinMode(fireSensePins[i],INPUT);
	}
}

void FireSensorArray::flip(){
	int tempPins[NUMFIRESENSORS];
	for(int i=1; i <= NUMFIRESENSORS; i++){
		tempPins[NUMFIRESENSORS-i] = fireSensePins[i-1];
	}
	for(int i=0; i < NUMFIRESENSORS; i++){
		fireSensePins[i] = tempPins[i];
	}
}

boolean FireSensorArray::isThereFire(){
	return (fireStrength()>FIRETHRESHOLD);
}

int FireSensorArray::fireAngle(){
  int curMax = 0;
  for(int i = 0; i < NUMFIRESENSORS; i++) {
	int reading = analogRead(fireSensePins[i]);
    if(reading > curMax) {
      curMax = reading;
    }
  }
  int angle = map((analogRead(fireSensePins[NUMFIRESENSORS-1]) - analogRead(fireSensePins[0])),-curMax, curMax,-45,45);
  if(curMax<15) angle=0; //No fire to read, don't give noise back as data
  return angle;
}

int FireSensorArray::fireStrength(){
  int curMax = 0;
  for(int i = 0; i < NUMFIRESENSORS; i++) {
    int reading = analogRead(fireSensePins[i]);
    if(reading > curMax) {
      curMax = reading;
    }
  }
  return curMax;
}

