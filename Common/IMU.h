#ifndef IMU_H
#define IMU_H

#include "drivers/hitechnic-superpro.h"

void initIMU() {
	HTSPBwriteStrobe(HTSPB, 1);
	HTSPBsetupIO(HTSPB, 0);
}

float getHeading() {
	return (360.0/255.0) * HTSPBreadIO(HTSPB, 255);
}

void resetArduino() {
	HTSPBwriteStrobe(HTSPB, 0);
	wait1Msec(1);
	HTSPBwriteStrobe(HTSPB, 1);
}

void turnWithGyro(int speed, int heading) {
	const int threshold = 1;
	while(getHeading() < heading-threshold || getHeading() > heading+threshold) {
		nxtDisplayCenteredTextLine(2, "%f", getHeading());
		turn(speed);
	}
	turn(0);
}

#endif /*IMU_H*/
