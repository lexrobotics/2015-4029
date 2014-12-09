#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  HTMotor)
#pragma config(Sensor, S2,     backUltra,      sensorSONAR)
#pragma config(Sensor, S3,     frontUltra,     sensorSONAR)
#pragma config(Motor,  mtr_S1_C1_1,     liftMotors,    tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     leftMotors,    tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     rightMotors,   tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_2,     scoop,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_1,     tilt,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_2,     nomotor,       tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C3_1,    grabber,              tServoStandard)
#pragma config(Servo,  srvo_S1_C3_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "JoystickDriver.c"
#include "../Common/basicMovement.c"
#define robotLength 12.0

void grabTube(){
	servo[grabber] = 100;
	pause(2);
}
void releaseTube(){
	servo[grabber] = 0;
	pause(1);
}
void tillBack(speed,sees){
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
void tillFront(speed,sees){
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
bool tooClose(threshold){
	//Sees if it is too close to drive in and get closer
	return ((SensorValue[frontUltra] + SensorValue[backUltra])/2 < threshold);
}
void parallel(speed){
	//Parrallells the robot. Threshold is the closeness of the sensors
	while(SensorValue[frontUltra] > SensorValue[backUltra]){
		turn(speed);
	}

	while(SensorValue[backUltra] > SensorValue[frontUltra]){
		turn(-speed);
	}
}
void knockdown(){
	//drives in turns knockdowns the kickstand and proceeds to end of wall
	float dist = SensorValue[frontUltra];
	turnDistance(-50, 90);
	moveDistance(100, dist/2);
	turnDistance(50, 90);
	parallel(50);
	tillFront(100,false);
}
void kickstand(){
	tillBack(50,true);
	parallel(50);
	moveDistance(50,3);
	if(SensorValue[frontUltra]>225){
		//position 1
		tillFront(-50,false);
		turnDistance(50,90);
		tillFront(-50,true);
		parrallel(50);
		tillBack(-100,false);
	}
	else{
		if(tooClose(10)){
			tilBack(100,false);
			return;
		}
		tillBack(50,false);
		moveDistance(50,3);
		knockdown();
	}
	//nxtDisplayCenteredTextLine(2,"angle: %f ", 90-radiansToDegrees(atan2(diff, robotLength)));
}

task main(){
	/*float robotLength = 29;
	while(true) {
		float diff = SensorValue[frontUltra] - SensorValue[backUltra];
		float angle = radiansToDegrees(atan(diff/robotLength));
		nxtDisplayCenteredTextLine(2,"angle: %f ", angle);
	}*/
	/*releaseTube();
	moveDistance(-100, 110);
	grabTube();
	turnDistance(100, 20);
	moveDistance(100, 85);
	turnDistance(100, 190);
	releaseTube();
	moveDistance(100, 35);*/
	kickstand();
	//moveDistance(100, 90);
}
