#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  HTMotor)
#pragma config(Sensor, S2,     backUltra,      sensorSONAR)
#pragma config(Sensor, S3,     frontUltra,     sensorSONAR)
#pragma config(Sensor, S4,     touch,          sensorTouch)
#pragma config(Motor,  mtr_S1_C1_1,     liftMotors,    tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     leftMotors,    tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     rightMotors,   tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     spinner,       tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_1,     liftStageOne,  tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C4_2,     liftStageTwo,  tmotorTetrix, openLoop, encoder)
#pragma config(Servo,  srvo_S1_C3_1,    rearUltraServo,       tServoStandard)
#pragma config(Servo,  srvo_S1_C3_2,    bucketTilt,           tServoStandard)
#pragma config(Servo,  srvo_S1_C3_3,    bucketGate,           tServoStandard)
#pragma config(Servo,  srvo_S1_C3_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_6,    grabber,              tServoContinuousRotation)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "JoystickDriver.c"
#include "drivers/hitechnic-irseeker-v2.h"
#include "../../Common/basicMovement.c"
#define robotLength 12.0

bool ultraReady = true;
int ultraAngle = 0;

void waitForUltra();

void grabTube(){
	servo[grabber] = 255;
	pause(1.3);
	servo[grabber] = 127;
}
task releaseTube(){
	servo[grabber] = 0;
	pause(0.9);
	servo[grabber] = 127;
}
void tillBack(int speed,bool sees){
	waitForUltra();
	//Takes in a speed and a sees boolean.
	//This ends when the back ultra sonic either sees something or doesn't
	//see something depending on the sees boolean
	move(speed);
	if(sees){
		while(SensorValue[backUltra] >225){};
	}
	else{
		while(SensorValue[backUltra] <225){};
	}
	move(0);
}

void tillBack(int speed,bool sees, int threshold){
	waitForUltra();
	//Takes in a speed and a sees boolean.
	//This ends when the back ultra sonic either sees something or doesn't
	//see something depending on the sees boolean
	move(speed);
	if(sees){
		while(SensorValue[backUltra] > threshold){};
	}
	else{
		while(SensorValue[backUltra] < threshold){};
	}
	move(0);
}

void tillFront(int speed, bool sees){
	waitForUltra();
	//Takes in a speed and a sees boolean.
	//This ends when the back ultra sonic either sees something or doesn't
	//see something depending on the sees boolean
	move(speed);
	if(sees){
		while(SensorValue[backUltra] >225){};
	}
	else{
		while(SensorValue[backUltra] <225){};
	}
	move(0);
}
bool tooClose(int threshold){
	waitForUltra();
	//Sees if it is too close to drive in and get closer
	return ((SensorValue[frontUltra] + SensorValue[backUltra])/2 < threshold);
}
/*
void parallel(int speed){
	//Parrallells the robot. Threshold is the closeness of the sensors
	while(SensorValue[frontUltra] > SensorValue[backUltra]){
		turn(speed);
	}

	while(SensorValue[backUltra] > SensorValue[frontUltra]){
		turn(-speed);
	}
}
*/

void parallel(int speed){
	waitForUltra();
	//Parrallells the robot. Threshold is the closeness of the sensors
	while(abs(SensorValue[frontUltra] - (SensorValue[backUltra] - 1)) > 0) {
		if(SensorValue[frontUltra] > SensorValue[backUltra] - 1){
			turn(-speed);
		}
		else if(SensorValue[frontUltra] < SensorValue[backUltra] - 1) {
			turn(speed);
		}
	}
	move(0);
}

task turnUltraTask() {
	ultraReady = false;
	int ZERO = 189;
	int scaled = ultraAngle * 255.0/180.0;

	servo[rearUltraServo] = ZERO - scaled;
	pause(0.4);
	ultraReady = true;
}

void waitForUltra() {
	move(0);
	while(!ultraReady);
}

// wrapper for turnUltraTask to retain backwards compat
void turnUltra(int angle) {
	ultraAngle = angle;
	StartTask(turnUltraTask);
}
void sweep(int afterAngle) {
	turn(30);
	float sum = SensorValue[backUltra];
	int ct = 1;
	while(SensorValue[backUltra] - sum/ct > -2){
		sum += SensorValue[backUltra];
		ct++;
	}
	move(0);
	turnDistance(50, afterAngle);
}
void tillBackWithFilter(int speed,bool sees){
	waitForUltra();
	//Takes in a speed and a sees boolean.
	//This ends when the back ultra sonic either sees something or doesn't
	//see something depending on the sees boolean
	move(speed);
	if(sees){
		while(true){
			while(SensorValue[backUltra] > 225){};
			int dist = SensorValue[backUltra];
			move(0);
			turnUltra(-1*atan2(45,dist)-10);
			pause(1);
			move(speed);
			if(SensorValue[backUltra]>225){
				turnUltra(0);
				while(SensorValue[backUltra] < 225){};
			}
			else{
				turnUltra(0);
				break;
			}

		}
	}
	else{
		while(SensorValue[backUltra] <225){};
	}
	move(0);
}

task init() {
	startTask(releaseTube);
	turnUltra(0);
}

task scoreAutoBall() {
	const int UPPER_LIFT_TARGET = 1.2 * 1440;
	const int LOWER_LIFT_TARGET = 21 * 280;
	nMotorEncoder[liftStageOne] = 0;
	nMotorEncoder[liftStageTwo] = 0;
	ClearTimer(T3);
	while(SensorValue[touch] != 1 || abs(nMotorEncoder[liftStageTwo]) < UPPER_LIFT_TARGET) {
	nxtDisplayCenteredTextLine(2, "%d", nMotorEncoder[liftStageOne]);
	if(SensorValue[touch] != 1)
			motor[liftStageOne] = 100;
		else
			motor[liftStageOne] = 0;
		if(abs(nMotorEncoder[liftStageTwo]) < UPPER_LIFT_TARGET)
			motor[liftStageTwo] = -100;
		else
			motor[liftStageTwo] = 0;
	}
	motor[liftStageOne] = 0;
	motor[liftStageTwo] = 0;
	nxtDisplayCenteredTextLine(2, "there");
	servo[bucketTilt] = 225;
	pause(1);
	servo[bucketGate] = 165;
	pause(0.5);
	servo[bucketGate] = 10;
	pause(0.5);
	servo[bucketTilt] = 120;
	pause(0.5);
	while(abs(nMotorEncoder[liftStageTwo]) > 0.7 * 1440) {
		nxtDisplayCenteredTextLine(2, "%d", nMotorEncoder[liftStageOne]);
		if(abs(nMotorEncoder[liftStageTwo]) > 0.7 * 1440)
			motor[liftStageTwo] = 100;
		else
			motor[liftStageTwo] = 0;
	}
	servo[bucketTilt] = 135;
	while(abs(nMotorEncoder[liftStageTwo]) > 0.4 * 1440) {
		nxtDisplayCenteredTextLine(2, "%d", nMotorEncoder[liftStageOne]);
		if(abs(nMotorEncoder[liftStageTwo]) > 0.4 * 1440)
			motor[liftStageTwo] = 100;
		else
			motor[liftStageTwo] = 0;
	}
	motor[liftStageTwo] = 0;
	motor[liftStageOne] = -100;
	pause(1);
	motor[liftStageOne] = 0;
	nxtDisplayCenteredTextLine(2, "done");
}

void Ramp(){
	// Navigate down the ramp and grab tube
	pause(0.5);
	startTask(init);
	moveDistancePID(-97, 0.02, 0);
	//StartTask(scoreAutoBall);
	grabTube();
	// bring tube to goal
	//turnDistancePID(40);
	turnDistance(100, 20);
	moveDistance(100, 10);
	turnDistance(100, 17);
	moveDistance(100, 60);
	turnDistance(100, 170);
	resetEncoders();
	turnUltra(90);
	while(SensorValue[backUltra] > 65) {
		move(-100);
	}
	startTask(releaseTube);
	//parallel and get in front of ramp
	turnUltra(0);
	turnDistance(100, 240);

	//moveDistance(100, 5);
	//tillFront(50,true);
	//pause(0.5);
	//parallel(50);
	//pause(0.5);

	//tillBack(-30,false, 60);
	//angle towards the wall and turn the ultra perpendicular to the wall
	//changed for speed
	moveDistance(-100, 50);
	turnDistance(100, 20);
	turnUltra(45);
	//get within range
	waitForUltra();
	tillBack(-70,true,36);
	turnUltra(0);
	waitForUltra();
	pause(0.2);
	parallel(40);
	pause(0.5);
	turnUltra(95);
	waitForUltra();
	tillBack(-50,true, 52);
	move(0);
	pause(0.3);
	//swivel and find tube
	sweep(17);

	//Use ultrasonics to calculate angle needed to reach second tube
	/*float d1 = SensorValue[backUltra] + 26 - 30.0;
	float d2 = SensorValue[frontUltra] + 17.8 - 20.0;
	int distFromTube = sqrt(pow(d1, 2) + pow(d2, 2));
	float angle = (180/PI) * atan(d2/d1);

	turnDistancePID(angle);
	*/
	/*move(0);
	while(true) {
		nxtDisplayCenteredTextLine(1, "f: %d, b: %d", SensorValue[frontUltra], SensorValue[backUltra]);
		nxtDisplayCenteredTextLine(2, "f: %d, b: %d", d1, d2);
		nxtDisplayCenteredTextLine(3, "a: %f", angle);
	}*/
	//Turn ultrasonic and wait to see tube in range
	pause(0.5);
	turnUltra(135);
	move(-40);
	while(SensorValue[backUltra]>30){};
	move(0);
	moveDistance(-50, 7);
	grabTube();
	turnDistance(-100, 5);
	moveDistance(100, 100);
	//return to parking goal
	/*turnUltra(0);
	turnDistance(50, 15);
	moveDistance(50, 12);
	pause(0.2);
	parallel(50);
	pause(0.2);
	turnUltra(90);
	waitForUltra();
	int x = SensorValue[backUltra];
	int y = SensorValue[frontUltra];
	float angle = atan((170.0 - y)/(365.0 - x)) * (180 / PI);
	turnDistance(100, angle);
	moveDistance(100, 36); */
	/*moveDistance(50, 12);
	turnDistance(50, 35);
	moveDistance(100, 70);*/
	turnDistance(100, 200);
	turnUltra(85);
	waitForUltra();
	while(SensorValue[backUltra] > 65) {
		move(-100);
	}
	move(0);
	startTask(releaseTube);
}

#ifndef AUTO_COMPETITION
task main() {
	servo[bucketGate] = 10;
	pause(0.5);
	servo[bucketTilt] = 255;
	Ramp();
	////turnUltra(0);

}
#endif
