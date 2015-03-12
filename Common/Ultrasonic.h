#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "Movement.h"

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
void tillSense(int speed, int angle, bool see_now, int threshold, tSensors sonar){
		//see_now describes whether the ultrasonic is currently within the threshold.
		//True is that it is within the threshold.
		//the robot will move in the specified way until the robot's ultrasonic no longer agrees with see_now.

		translateRT(speed,angle);
		while(((SensorValue[sonar]<threshold)==see_now) || (SensorValue[sonar] == 255)){}

		translateRT(0, 0);
}

void changeDetection(int speed, int angle, int jumpThreshold, tSensors sonar) {
	int dist = SensorValue[sonar];

	while (abs(dist - SensorValue[sonar]) < jumpThreshold) {
		translateRT(speed, angle);
		dist = SensorValue[sonar];
	}

	translateRT(0, 0);
}
void findWall(){
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
void parallel(int speed, int threshold, tSensors sensorA, tSensors sensorB){
	if(sensorValue[sensorA] == 255 && sensorValue[sensorB]){ findWall(); }
	int valA = SensorValue[sensorA];
	int valB = SensorValue[sensorB];
	while (abs(valA - valB) >= threshold) {
		pause(0.02);
		valA = SensorValue[sensorA];
 		valB = SensorValue[sensorB];
 		//turn((valA - valB) * 0.20 * speed);
		turn(speed * (valA>valB ? 1 : -1));
	}
	turn(0);
}

void incrementalParallel(int speed, int threshold, tSensors sensorA, tSensors sensorB) {
	if(sensorValue[sensorA] == 255 && sensorValue[sensorB]){ findWall(); }
	int valA = SensorValue[sensorA];
	int valB = SensorValue[sensorB];
	int count =0;
	while (count<=3) {
		if(sensorValue[sensorA] == 255 && sensorValue[sensorB]){ findWall(); }
		if(abs(valA - valB) <= threshold){
			count++;
		}
		else{
			count=0;
			turnDistance(speed * (valA>valB ? 1 : -1), 1);
		}
		pause(0.02);
		valA = SensorValue[sensorA];
 		valB = SensorValue[sensorB];
 		//turn((valA - valB) * 0.20 * speed);
		//turnDistance(speed * (valA>valB ? 1 : -1), 1);
	}
	turn(0);
}

void lateralCenter(int speed, int angle, int threshold, tSensors sensorA, tSensors sensorB){
	turnUltra(0,angle);
	turnUltra(1,angle);
	pause(0.5);
	int valA = SensorValue[sensorA];
	int valB = SensorValue[sensorB];
	while (abs(valA - valB) > threshold) {
		valA = SensorValue[sensorA];
 		valB = SensorValue[sensorB];
		move(speed * (valA>valB ? 1 : -1));
	}
	turn(0);
}

int detectPosition(){
	float avg = 0;
	const int READINGS = 30;
	int READINGSARR[30];
	for(int i=0; i<READINGS; i++) {
		READINGSARR[i]=USreadDist(frontUS);
		avg += USreadDist(frontUS);
		wait1Msec(5):
	}
	avg /= READINGS;
	float filtered_avg = 0;
	int threshold = 30;
	for(int i=0; i<READINGS; i++){
		if(abs(avg - READINGSARR[i])<threshold){
			filtered_avg += READINGSARR[i];
		}
	}
	filtered_avg = filtered_avg/30;
	//	while(true){
	//	nxtDisplayCenteredTextLine(1,"front: %f", filtered_avg);
	//	nxtDisplayCenteredTextLine(2,"back: %f", avg);
	//	wait1Msec(5);
	//}

	if(filtered_avg < 70)
		return 3;
	else if(100 > filtered_avg && filtered_avg > 70)
		return 1;
	else
		return 2;
}
#endif /* ULTRASONIC_H */
