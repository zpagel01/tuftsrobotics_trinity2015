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
  long weightedAngle = 0;
  long total = 0;
  for(int i = 0; i < NUMFIRESENSORS; i++) {
	long reading = analogRead(fireSensePins[i]);
        /*
        Serial.print("Sensor ");
        Serial.print(i);
        Serial.print(" reads ");
        Serial.println(reading);
        */
    if(reading > curMax) {
      curMax = reading;
    }
	long sensorAngle = map(i,0,NUMFIRESENSORS-1,90,-90);
        //Serial.print("Sensor's angle: ");
        //Serial.println(sensorAngle);
	long weight = reading * reading;
	total += weight;
	weightedAngle += sensorAngle * weight;
  }
  weightedAngle /= total;
  if(curMax<FIRETHRESHOLD) weightedAngle=0; //No fire to read, don't give noise back as data
  /*
  Serial.print("Angle: ");
  Serial.println(weightedAngle);
  */
  return weightedAngle;
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

