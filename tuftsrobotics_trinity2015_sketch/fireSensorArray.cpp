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
  int curMax = -1;
  int curMaxI = -1;
  long weightedAngle = 0;
  long total = 0;
  
  for(int i = 0; i < NUMFIRESENSORS; i++) {
    long reading = analogRead(fireSensePins[i]);
    if(reading > curMax) {
      curMax = reading;
      curMaxI = i;
    }
  }
  for (int i=curMaxI-1; i<=curMaxI+1; i++)
  {
    if (i>=0 && i<NUMFIRESENSORS){
      
        long reading = analogRead(fireSensePins[i]);
    	long sensorAngle = map(i,0,NUMFIRESENSORS-1,60,-60);
        long weight = reading;
	
        if (weight>FIRETHRESHOLD){
          total += weight;
          weightedAngle += sensorAngle * weight;
        }
    }
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

