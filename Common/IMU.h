#ifndef IMU_H
#define IMU_H

#include "drivers/hitechnic-superpro.h"

void initIMU() {
	HTSPBwriteStrobe(HTSPB, 1);
	HTSPBsetupIO(HTSPB, 0);
}

float getHeading() {
	return (360.0/255.0) * HTSPBreadIO(HTSPB, 255) - 180.0;
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
