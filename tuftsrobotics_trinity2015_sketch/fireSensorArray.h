#ifndef FIRESENSORARRAY_H
#define FIRESENSORARRAY_H
#include "Arduino.h"

#define NUMFIRESENSORS  5
#define FIRETHRESHOLD   70

class FireSensorArray{
	public:
		FireSensorArray();
		void attach(int pins[NUMFIRESENSORS]);
		void flip();
		boolean isThereFire();
		int fireAngle();
                int fireStrength();
	private:
		int fireSensePins[NUMFIRESENSORS];
};

#endif
