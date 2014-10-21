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

void grabTube(){
	servo[grabber] = 100;
	pause(2);
}
void releaseTube(){
	servo[grabber] = 0;
	pause(1);
}
void kickstand(){
	float robotLength = 12;
	move(50);
	while(SensorValue[backUltra] >225){
	}
	move(0);
	turn(-10);
	while(SensorValue[frontUltra] >225){}
	if(SensorValue[frontUltra] < 20){
		moveDistance(100, 50);
		return;
	}
	pause(0.1);

	float diff = SensorValue[frontUltra] - SensorValue[backUltra];
	float angle = radiansToDegrees(atan(diff/robotLength));
	if(angle >= 0 && angle <= 90) {
		turnDistance(-100, angle);
	}
	else if(angle >= 270 && angle <= 360) {
		turnDistance(100, 360-angle);
	}
	//pause(100);
	//PARRALLED
	move(0);
	move(-50);
	while(SensorValue[backUltra] < 200) {};
	moveDistance(50,3);
	turnDistance(-50, 90);
	moveDistance(100, (robotLength + diff)/2);
	turnDistance(50, 90);
	while(SensorValue[backUltra] > 100){move(50)};
	move(0);
	diff = SensorValue[frontUltra] - SensorValue[backUltra];
	angle = radiansToDegrees(atan(diff/robotLength));
	if(angle >= 0 && angle <= 90) {
		turnDistance(-100, angle);
	}
	else if(angle >= 270 && angle <= 360) {
		turnDistance(100, 360-angle);
	}
	moveDistance(100, 50);
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
