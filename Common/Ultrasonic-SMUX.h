#ifndef ULTRASONIC_SMUX_H
#define ULTRASONIC_SMUX_H

#include "Movement.h"

#include "drivers/hitechnic-sensormux.h"
#include "drivers/lego-ultrasound.h"
#include "drivers/hitechnic-irseeker-v2.h"

const tMUXSensor clampUS = msensor_S3_1;
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
void findWall(tMUXSensor frontUltra, tMUXSensor rearUltra){
	while(SensorValue[frontUltra] == 255 && SensorValue[rearUltra] == 255){
		for(int i = 0; i<90; i++) {
			turnUltra(0, i);
			turnUltra(1, i);
			pause(0.05);
			if(SensorValue[frontUltra] != 255) {
				turnDistance(50, i);
				break;
			}
			if(SensorValue[rearUltra] != 255) {
				turnDistance(-50, i);
				break;
			}
		}
	}
	turnUltra(0, 0);
	turnUltra(1, 0);
	pause(0.5);
}

/*
Hybrid functions
*/
// hold on to your socks clive
void filteredTillSense(int speed, int angle, bool see_now, int threshold, float a = 0.1, tMUXSensor sonar){
		//see_now describes whether the ultrasonic is currently within the threshold.
		//True is that it is within the threshold.
		//the robot will move in the specified way until the robot's ultrasonic no longer agrees with see_now.
		float value = USreadDist(sonar);
		translateRT(speed,angle);
		//while(((value<threshold)==see_now) || ((value == 255) && (!see_now))){
		while(true){
			value = a * USreadDist(sonar) + (1-a) * value;
			writeDebugStreamLine("%d", USreadDist(sonar));
			wait10Msec(1);
		}
		writeDebugStreamLine("%d", value);
	  	writeDebugStreamLine("DONE!");
		translateRT(0, 0);
}

/*
Hybrid functions
*/
void tillSense(int speed, int angle, bool see_now, int threshold, tMUXSensor sonar){
		//see_now describes whether the ultrasonic is currently within the threshold.
		//True is that it is within the threshold.
		//the robot will move in the specified way until the robot's ultrasonic no longer agrees with see_now.

		translateRT(speed,angle);
		while(((USreadDist(sonar)<threshold)==see_now) || ((USreadDist(sonar) == 255) && (!see_now))){
			writeDebugStreamLine("%d", USreadDist(sonar));
						wait10Msec(1);
		}
		writeDebugStreamLine("%d", USreadDist(sonar));
	  	writeDebugStreamLine("DONE!");
		translateRT(0, 0);
}

void repeatedTillSense(int speed, int angle, bool see_now, int threshold, tMUXSensor sonar){
		//see_now describes whether the ultrasonic is currently within the threshold.
		//True is that it is within the threshold.
		//the robot will move in the specified way until the robot's ultrasonic no longer agrees with see_now.
		int ct = 0;

		translateRT(speed,angle);
		while(true){
			if((USreadDist(sonar) > threshold) == see_now) {
				ct++;
			}
			else {
				ct = 0;
			}
			if(ct > 10) {
				break;
			}
			wait1Msec(5);
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
	if(USreadDist(sensorA) == 255 && USreadDist(sensorB)){ findWall(sensorA, sensorB); }
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
	if(USreadDist(sensorA) == 255 && USreadDist(sensorB)){ findWall(sensorA,sensorB); }
	int valA = USreadDist(sensorA);
	int valB = USreadDist(sensorB);
	int count =0;
	while (count<=3) {
		if(USreadDist(sensorA) == 255 && USreadDist(sensorB)){ findWall(sensorA, sensorB); }
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

int detectPosition(){
	float avg = 0;
	float READINGS = 50.0;
	int READINGSARR[50];
	for(int i=0; i<READINGS; i++) {
		READINGSARR[i]=USreadDist(frontUS);
		avg += USreadDist(frontUS);
		wait1Msec(5);
	}
	avg /= READINGS;
	float filtered_avg = 0;
	int threshold = 20;
	for(int i=0; i<READINGS; i++){
		if(abs(avg - READINGSARR[i])<threshold){
			filtered_avg += READINGSARR[i];
		}
		else {
			READINGS--;
		}
	}
	filtered_avg = filtered_avg/READINGS;
	//	while(true){
	//	nxtDisplayCenteredTextLine(1,"front: %f", filtered_avg);
	//	nxtDisplayCenteredTextLine(2,"back: %f", avg);
	//	wait1Msec(5);

	if(filtered_avg < 70)
		return 3;
	else if(100 > filtered_avg && filtered_avg > 70)
		return 1;
	else
		return 2;

	//if(filtered_avg > 110){
	//	return 2;
	//}
	//else if( (3 < irsector) && (irsector < 7 )){
	//	return 3;
	//}
	//else {
	//		return 1;
	//}
}

int detectPosition(int irsector){
	float avg = 0;
	const float READINGS = 50.0;
	int READINGSARR[50];
	for(int i=0; i<READINGS; i++) {
		READINGSARR[i]=USreadDist(frontUS);
		avg += USreadDist(frontUS);
		wait1Msec(5):
	}
	avg /= READINGS;
	float filtered_avg = 0;
	int threshold = 20;
	for(int i=0; i<READINGS; i++){
		if(abs(avg - READINGSARR[i])<threshold){
			filtered_avg += READINGSARR[i];
		}
	}
	filtered_avg = filtered_avg/READINGS;
	//	while(true){
	//	nxtDisplayCenteredTextLine(1,"front: %f", filtered_avg);
	//	nxtDisplayCenteredTextLine(2,"back: %f", avg);
	//	wait1Msec(5);

	//if(filtered_avg < 70)
	//	return 3;
	//else if(100 > filtered_avg && filtered_avg > 70)
	//	return 1;
	//else
	//	return 2;

	if(filtered_avg > 110){
		return 2;
	}
	else if( (3 < irsector) && (irsector < 7 )){
		return 3;
	}
	else {
			return 1;
	}
}
#endif /* ULTRASONIC_SMUX_H */
