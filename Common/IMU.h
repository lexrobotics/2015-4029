#ifndef IMU_H
#define IMU_H

#include "drivers/hitechnic-superpro.h"

float heading_offset = 0;

void initIMU() {
	HTSPBwriteStrobe(HTSPB, 1);
	HTSPBsetupIO(HTSPB, 0);
}

float getHeading() {
	float h = (360.0/255.0) * HTSPBreadIO(HTSPB, 255) - 180.0;
	return h - heading_offset;
}

void resetIMUOffset() {
	heading_offset = getHeading();
}

int getAccelX() {
	// returns units in 1/8th Gs
	return (HTSPBreadIO(HTSPB, 255) >> 4) - 8;
}

int getAccelY() {
	// returns units in 1/8th Gs
	return (HTSPBreadIO(HTSPB, 255) & 0b00001111) - 8;
}

void resetArduino() {
	HTSPBwriteStrobe(HTSPB, 0);
	wait1Msec(1);
	HTSPBwriteStrobe(HTSPB, 1);
}

//void turnWithGyro(int speed, float heading) {
//	const float threshold = 0.5;
//	speed = 130;
//	//getHeading() < heading-threshold || getHeading() > heading+threshold
//	while(speed > 30) {
//		float measuredHeading = abs(getHeading());
//		speed = 130 - (measuredHeading/heading) * 100.0;
//		writeDebugStreamLine("heading: %f, speed: %d", measuredHeading, speed);
//		turn(speed);
//		wait1Msec(5);
//	}
//	turn(0);
//	PlaySound(soundBeepBeep);
//	while(true);
//}

#endif /*IMU_H*/
