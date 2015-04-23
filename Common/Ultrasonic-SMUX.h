#ifndef ULTRASONIC_SMUX_H
#define ULTRASONIC_SMUX_H

#include "Movement.h"

#include "drivers/hitechnic-sensormux.h"
#include "drivers/lego-ultrasound.h"
#include "drivers/hitechnic-irseeker-v2.h"

const tMUXSensor frontUS = msensor_S3_1;
const tMUXSensor clampUS = msensor_S3_3;
const tMUXSensor rearUS = msensor_S3_4;
const tMUXSensor irSeeker = msensor_S3_2;
void irTillSensePeak(int speed){
	int acS1=0, acS2=0, acS3=0, acS4=0, acS5 = 0;
	int irVals[5];

	int lastreading = acs3;
	int count = 0;
	translateRTHeading(speed, 0);
	while(acS3 < 20){
			writeDebugStreamLine("1: %d, 2: %d, 3: %d, 4: %d, 5: %d // init", acS1, acS2, acS3, acS4, acS5);
			if (!HTIRS2readAllACStrength(irSeeker, acS1, acS2, acS3, acS4, acS5 ))
      	break;
     wait1Msec(5);
	}
	while(count < 5) {


		if (!HTIRS2readAllACStrength(irSeeker, acS1, acS2, acS3, acS4, acS5 ))
        break; // I2C read error occurred

    if(hugeBump) {
    	count = 0;
    	hugeBump = false;
    }

    if(acS3>lastreading && acS3 != 0 && acS3 != lastreading ){
    	count=0;
		}
		else if(acS3 != lastreading && USreadDist(frontUS) != 100){
			count++;
		}
		lastreading = acS3;
   	writeDebugStreamLine("1: %d, 2: %d, 3: %d, 4: %d, 5: %d, ct: %d // actual", acS1, acS2, acS3, acS4, acS5, count);
		wait1Msec(5);
	}
	translating = false;
	PlaySound(soundFastUpwardTones);
}

//Turns the ultrasonic indicated to the angle indicated.
//Asynchronous, so you have to wait a bit.
void turnUltra(int servo_index, int angle) {
	int scaled = angle * 256.0/180.0;
	int ZERO;
	switch(servo_index) {
		case 0:
			ZERO = 127;
			servo[frontTurret] = ZERO + scaled;
			break;
		case 1:
			ZERO = 95;
			servo[rearTurret] =  ZERO + scaled ;
			break;
	}
}

//void binaryTillSense(int speed, int angle, int threshold, tMUXSensor sonar){
//	translateRT(0,0);
//	int ct=0;
//	int index =0;
//	int readings = 10;
//	int readingsarr[10];
//	int currentreading;
//	int lastreading = USreadDist(sonar);
//	wait10Msec(1);
//	float avg = USreadDist(sonar) ;
//	int i;
//	for(i=0;i<readings;i++){
//		readingsarr[i] = avg;
//	}
//	translateRT(speed,angle);
//	int diff;
//	int initcount=0;
//	while(ct<10){
//		currentreading = USreadDist(sonar);
//		diff = currentreading;
//		if( abs(avg - diff) > threshold && initcount>10){
//			ct++;
//		}
//		else{
//			ct=0;
//			avg = ((avg * readings) - readingsarr[index] + diff) / readings ;
//			readingsarr[index] = diff;
//			lastreading = currentreading;
//			index = (index + 1)%readings;
//		}
//		wait10Msec(1);
//		initcount++;
//	}
//	move(0);
//}

//void binaryTillSense(int speed, int angle, int threshold, tMUXSensor sonar){
//	translateRT(0,0);
//	int ct=0;
//	int index =0;
//	float readings = 20.0;
//	float readingsarr[20];
//	for(int i=0;i<readings;i++){
//		readingsarr[i] = 0;
//	}
//	int currentreading;
//	float avg = 0 ;
//	translateRT(speed,angle);
//	int initcount=0;
//	while(ct<10){
//		currentreading = USreadDist(sonar);
//		writeDebugStreamLine("read: %d avg: %f", currentreading, avg);
//		if( abs(avg - currentreading) > threshold && initcount>20){
//			ct++;
//		}
//		else{
//			ct=0;
//			avg = ((avg * readings) - readingsarr[index] + currentreading) / readings ;
//			readingsarr[index] = currentreading;
//			index = (index + 1)%((int)readings);
//		}
//		wait10Msec(1);
//		initcount++;
//	}
//	move(0);
//}

void binaryTillSense(int speed, int angle, int threshold, tMUXSensor sonar){
	translateRT(0,0);
	int ct=0;
	int ind =0;
	float readings = 20.0;
	float readingsarr[20];
	for(int i=0;i<readings;i++){
		readingsarr[i] = 0;
	}
	int currentreading;
	float avg = 0 ;
	translateRT(speed,angle);
	int initcount=0;
	while(ct<10){
		currentreading = USreadDist(sonar);
		writeDebugStreamLine("read: %d avg: %f", currentreading, avg);
		if( abs(avg - currentreading) > threshold && initcount>20){
			ct++;
		}
		else{
			ct=0;
			readingsarr[ind] = currentreading;
			float sum = 0;
			for(int i=0; i<readings; i++) {
				sum+=readingsarr[i];
			}
			avg = sum/readings;
			ind = (ind + 1)%((int)readings);
		}
		wait10Msec(1);
		initcount++;
	}
	move(0);
}

void binaryTillSenseHeading(int speed, int angle, int threshold, tMUXSensor sonar){
	translateRT(0,0);
	int ct=0;
	int index =0;
	float readings = 20.0;
	float readingsarr[20];
	for(int i=0;i<readings;i++){
		readingsarr[i] = 0;
	}
	int currentreading;
	float avg = 0 ;
	translateRTHeading(speed, angle);
	int initcount=0;
	while(ct<10){
		currentreading = USreadDist(sonar);
		writeDebugStreamLine("read: %d avg: %f", currentreading, avg);
		if( abs(avg - currentreading) > threshold && initcount>20){
			ct++;
		}
		else{
			ct=0;
			readingsarr[index] = currentreading;
			float sum = 0;
			for(int i=0; i<readings; i++) {
				sum+=readingsarr[i];
			}
			avg = sum/readings;
			index = (index + 1)%((int)readings);
		}
		wait10Msec(1);
		initcount++;
	}
	translating = false;
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
			wait10Msec(1);
		}
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
						wait10Msec(1);
		}
		translating = false;
			translateRT(0, 0);
}

void tillSenseHeading(int speed, int angle, bool see_now, int threshold, tMUXSensor sonar){
		//see_now describes whether the ultrasonic is currently within the threshold.
		//True is that it is within the threshold.
		//the robot will move in the specified way until the robot's ultrasonic no longer agrees with see_now.

		translateRTHeading(speed,angle);
		pause(0.1);
		while(((USreadDist(sonar)<threshold)==see_now) || ((USreadDist(sonar) == 255) && (!see_now))){
						wait10Msec(1);
		}
		translating = false;
			//translateRT(0, 0);
}


void repeatedTillSenseHeading(int speed, int angle, bool see_now, int threshold, tMUXSensor sonar){
		//see_now describes whether the ultrasonic is currently within the threshold.
		//True is that it is within the threshold.
		//the robot will move in the specified way until the robot's ultrasonic no longer agrees with see_now.
		int ct = 0;
		translateRTHeading(speed,angle);
		while(true){
			int val = USreadDist(sonar);
			if((val > threshold) == see_now) {
				ct++;
				playSound(soundBlip);
			}
			else {
				ct = 0;
			}
			if(ct > 10) {
				break;
			}
			wait1Msec(5);
		}
		move(0);
}

void repeatedTillSense(int speed, int angle, bool see_now, int threshold, tMUXSensor sonar){
		//see_now describes whether the ultrasonic is currently within the threshold.
		//True is that it is within the threshold.
		//the robot will move in the specified way until the robot's ultrasonic no longer agrees with see_now.
		int ct = 0;

		translateRT(speed,angle);
		while(true){
			int val = USreadDist(sonar);
			if((val > threshold) == see_now) {
				ct++;
				playSound(soundBlip);
			}
			else {
				ct = 0;
			}
			if(ct > 10) {
				break;
			}
			wait1Msec(5);
		}
		move(0);
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

float stdDev(int *a, int avg, float len) {

}

int detectPosition(){
	float avg = 0;
	float READINGS = 50.0;
	int READINGSARR[50];
	for(int i=0; i<READINGS; i++) {
		int sensevalue = USreadDist(frontUS);

		writeDebugStreamLine("sense value: %d", sensevalue);
		READINGSARR[i]=sensevalue;
		avg += sensevalue;
		wait1Msec(5);
	}
	avg /= READINGS;

	float sum=0;
	for(int i=0; i<READINGS; i++) {
		float t = pow(READINGSARR[i] - avg, 2);
		writeDebugStreamLine("%f", t);
		sum += t;
	}
	writeDebugStreamLine("sum: %f", sum);
	sum /= READINGS;
	float threshold = sqrt(sum);

	writeDebugStream("stdev: %f", threshold);
	float filtered_avg = 0;
	float filteredReadings = 0;
	for(int i=0; i<READINGS; i++){
		if(abs(avg - READINGSARR[i])<=threshold){
			filtered_avg += READINGSARR[i];
			filteredReadings++;
		}

	}
	filtered_avg = filtered_avg/filteredReadings;
	//	while(true){
	//	nxtDisplayCenteredTextLine(1,"front: %f", filtered_avg);
	//	nxtDisplayCenteredTextLine(2,"back: %f", avg);
	//	wait1Msec(5);

	writeDebugStreamLine("filtered avg: %d; stdev: %f", filtered_avg, threshold);
	if(filtered_avg < 70)
		return 3;
	else if(110 > filtered_avg && filtered_avg > 70)
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

#endif /* ULTRASONIC_SMUX_H */
