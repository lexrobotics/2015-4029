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
#include "../../Common/basicMovement.c"
#define robotLength 12.0

void grabTube(){
	servo[grabber] = 100;
	pause(2);
}
void releaseTube(){
	servo[grabber] = 0;
	pause(1);
}

int tillFront(int speed, bool sees){
	move(speed);
	int encoder_val = nMotorEncoder[leftMotors];
	if(sees){
		while((SensorValue[frontUltra] > 225)){};
	}
	else{
			while((SensorValue[frontUltra] < 225)){};
	}
	move(0);
	return (nMotorEncoder[leftMotors] - encoder_val);
}

void optionOne(){
	tillFront(-100, false);
	turnDistance(100, 90);
	moveDistance(100, 50);
}

void tillBack(int speed, bool sees){
	//Takes in a speed and a sees boolean.
	//This ends when the back ultra sonic either sees something or doesn't
	//see something depending on the sees boolean
	move(speed);
	if(sees){
		while((SensorValue[backUltra] > 245)){};
	}
	else{
			while((SensorValue[backUltra] < 245)){};
	}
	move(0);
}

void halt(){
	move(0);
	pause(1000);
}

bool driveTimeout(int speed){
	int center_length = 27;
	moveDistance(speed, center_length/2.5);
	if ((SensorValue(frontUltra) < 225)){
		return false;
	}
	return true;
}

bool checkOne(){
	return driveTimeout(100);
}

bool tooClose(int threshold){
	//Sees if it is too close to drive in and get closer
	return ((SensorValue[frontUltra] + SensorValue[backUltra])/2 < threshold);
}
void getFrontInRange(){
	//Turn until the front ultrasonic sees the wall
	turn(10);
	while(SensorValue[frontUltra] >225){};
	turn(0);
}
void parallel(int threshold, int speed){
	//Parrallells the robot. Threshold is the closeness of the sensors
	while(SensorValue[frontUltra] < SensorValue[backUltra] ){
		turn(speed);
	}
	while(SensorValue[frontUltra] > SensorValue[backUltra]){
		turn(-speed);
	}
}
void knockdown(){
	//drives in turns knockdowns the kickstand and proceeds to end of wall
	float dist = SensorValue[frontUltra];
	turnDistance(-100, 90);
	moveDistance(100, (dist)/6);
	turnDistance(100, 90);
	move(100);
	while(SensorValue[frontUltra]<225){};
}
void kickstand(){
	tillBack(100, true);
	moveDistance(100,5);
	parallel(1,75);

	//if (!checkOne()){\
	if(true){
		//getFrontInRange();
		//halt();
		if(tooClose(10)){
			tillBack(100,false);
			return;
		}
	//pause(100);
	//PARRALLED
		tillBack(-50,false);
		moveDistance(50,2);
		knockdown();
	}
	else{
		optionOne();
	}
	//nxtDisplayCenteredTextLine(2,"angle: %f ", 90-radiansToDegrees(atan2(diff, robotLength)));
}

void Ramp(){
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

#ifndef AUTO_COMPETITION
task main() {
	Ramp();
}
#endif
