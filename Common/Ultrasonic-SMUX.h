#ifndef ULTRASONIC_SMUX_H
#define ULTRASONIC_SMUX_H

#include "Movement.h"

#include "drivers/hitechnic-sensormux.h"
#include "drivers/lego-ultrasound.h"

const tMUXSensor frontUS = msensor_S3_3;
const tMUXSensor rearUS = msensor_S3_4;

//Turns the ultrasonic indicated to the angle indicated.
//Asynchronous, so you have to wait a bit.
void turnUltra(int servo_index, int angle) {
	int scaled = angle * 256.0/180.0;
	int ZERO;
	switch(servo_index) {
		case 0:
			ZERO = 157;
			servo[frontTurret] = ZERO - scaled;
			break;
		case 1:
			ZERO = 95;
			servo[rearTurret] =  ZERO + scaled ;
			break;
	}
}

/*
Hybrid functions
*/
void tillSense(int speed, int angle, bool see_now, int threshold, tMUXSensor sonar){
		translateRT(speed,angle);
		while((USreadDist(sonar)<threshold)==see_now || (USreadDist(sonar) == 255 && !see_now)){
			writeDebugStreamLine("%d",USreadDist(sonar));

		}

		translateRT(0, 0);
}

void changeDetection(int speed, int angle, int jumpThreshold, tMUXSensor sonar) {
	int dist = USreadDist(sonar);

	while (abs(dist - USreadDist(sonar)) < jumpThreshold) {
		translateRT(speed, angle);
		dist = USreadDist(sonar);
	}

	translateRT(0, 0);
}

void parallel(int speed, int threshold, tMUXSensor sensorA, tMUXSensor sensorB){
	if(USreadDist(sensorA) == 255 && USreadDist(sensorB)){ findWall(); }
	int valA = USreadDist(sensorA);
	int valB = USreadDist(sensorB);
	while (abs(valA - valB) >= threshold) {
		pause(0.02);
		valA = USreadDist(sensorA);
 		valB = USreadDist(sensorB);
 		//turn((valA - valB) * 0.20 * speed);
		turn(speed * (valA>valB ? 1 : -1));
	}
	turn(0);
}

void incrementalParallel(int speed, int threshold, tMUXSensor sensorA, tMUXSensor sensorB) {
	if(USreadDist(sensorA) == 255 && USreadDist(sensorB)){ findWall(); }
	int valA = USreadDist(sensorA);
	int valB = USreadDist(sensorB);
	int count =0;
	while (count<=3) {
		if(USreadDist(sensorA) == 255 && USreadDist(sensorB)){ findWall(); }
		if(abs(valA - valB) <= threshold){
			count++;
		}
		else{
			count=0;
			turnDistance(speed * (valA>valB ? 1 : -1), 1);
		}
		pause(0.02);
		valA = USreadDist(sensorA);
 		valB = USreadDist(sensorB);
 		//turn((valA - valB) * 0.20 * speed);
		//turnDistance(speed * (valA>valB ? 1 : -1), 1);
	}
	turn(0);
}

void lateralCenter(int speed, int angle, int threshold, tMUXSensor sensorA, tMUXSensor sensorB){
	turnUltra(0,angle);
	turnUltra(1,angle);
	pause(0.5);
	int valA = USreadDist(sensorA);
	int valB = USreadDist(sensorB);
	while (abs(valA - valB) > threshold) {
		valA = USreadDist(sensorA);
 		valB = USreadDist(sensorB);
		move(speed * (valA>valB ? 1 : -1));
	}
	turn(0);
}
#endif /* ULTRASONIC_SMUX_H */
