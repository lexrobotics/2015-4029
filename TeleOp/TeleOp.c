#pragma config(Hubs,  S1, HTMotor,  HTMotor,  none,     none)
#pragma config(Hubs,  S2, HTMotor,  HTServo,  HTServo,  none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ,               sensorI2CMuxController)
#pragma config(Sensor, S3,     ultra1,         sensorSONAR)
#pragma config(Sensor, S4,     ultra0,         sensorSONAR)
#pragma config(Motor,  mtr_S1_C1_1,     motorFrontLeft, tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     motorBackLeft, tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     harvester,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     conveyor,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_1,     motorBackRight, tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S2_C1_2,     motorFrontRight, tmotorTetrix, openLoop, encoder)
#pragma config(Servo,  srvo_S2_C2_1,    lift1,                tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C2_2,    lift2,                tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C2_3,    grabber,              tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C2_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_6,    servo6,               tServoNone)
#pragma config(Servo,  srvo_S2_C3_1,    servo7,               tServoNone)
#pragma config(Servo,  srvo_S2_C3_2,    servo8,               tServoNone)
#pragma config(Servo,  srvo_S2_C3_3,    servo9,               tServoNone)
#pragma config(Servo,  srvo_S2_C3_4,    servo10,              tServoNone)
#pragma config(Servo,  srvo_S2_C3_5,    servo11,              tServoNone)
#pragma config(Servo,  srvo_S2_C3_6,    servo12,              tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/*
TeleOp.c
Implementing Mecanum drive capabilities.

LastUpdatedOn: 1/12
LastUpdatedBy: Clive, David
*/

/*
MOUNTING THE WHEELS:
-----Critically, the rollers must be parallel to the slashes in the diagram.-----
-----Or maybe not so critically??-----
[\\]   [fwd]    [//]
[\\]            [//]
[\\]            [//]

[left]       [right]

[//]            [\\]
[//]            [\\]
[//]   [back]   [\\]
*/

#include "JoystickDriver.c"
#include "../common/Movement.h"

const int WINCHSTOP = 11;

float normalize10(float x){
	if(abs(x) < 10) return 0;
	return x;
}

int max(int a, int b){
	if(a > b) return a;
	return b;
}

void rotate(float &x, float &y, float t){
	float oldx = x, oldy = y;
	x = oldx * cos(t) - oldy * sin(t);
	y = oldx * sin(t) + oldy * cos(t);
}

/*
For rotate-while-translating: It's been done! https://www.youtube.com/watch?v=sM8cixsE5fo
t = ??? * currentOrientation; //Orientation is probably best measured with gyro; you COULD do it with integral(vRot) though.
[ vFwd  ] = [ cos(t)  -sin(t) ] * [ vFwd  ] //depends on correct directionality of rotation, may have to switch +/-sin
[ vSide ]   [ sin(t)   cos(t) ]   [ vSide ]
Rotation matrix, which changes the requested vFwd and vSide so they are corrected for rotation. (keeps nonrotating frame of reference)
Random idea: https://www.youtube.com/watch?v=igaGWlMFdSw
*/

void init() {
	servo[lift1] = WINCHSTOP;
	servo[lift2] = WINCHSTOP;
}

task arm() {
  getJoystickSettings(joystick);
	while(true) {
		if(joy2Btn(6)) {
			motor[conveyor] = 100;
		}
		else if(Joy2Btn(8)) {
			motor[conveyor] = -100;
		}
		else {
			motor[conveyor] = 0;
		}
		if(joy1Btn(5)) {
			servo[grabber] = 255;
		}
		else if(joy1Btn(7)) {
			servo[grabber] = 0;
		}
		else {
			servo[grabber] = 127;
		}
		if(joy1Btn(6)) {
			motor[harvester] = 75;
		}
		else if(joy1Btn(8)) {
			motor[harvester] = -75;
		}
		else {
			motor[harvester] = 0;
		}
		if(joy2Btn(5)) {
			servo[lift1] = 0;
			servo[lift2] = 0;
		}
		else {
			servo[lift1] = WINCHSTOP;
			servo[lift2] = WINCHSTOP;
		}
	}
}

task main(){
  int x1, y1, x2, y2;
  bool harvesting = false;
  bool belting = false;
  init();
	waitForStart();
	StartTask(arm);
  while(true){
  	getJoystickSettings(joystick);

  	//AUGMENTED-TANK JOYSTICKING SYSTEM
	  x1 = normalize10(joystick.joy1_x1);
	  x2 = normalize10(joystick.joy1_x2);
	  y1 = normalize10(joystick.joy1_y1);
	  y2 = normalize10(joystick.joy1_y2);

	  float JoyToWheel = 95.0 / max(max(max(abs(y2 + x2),abs(y1 - x1)),max(abs(y2 - x1),abs(y2 + x2))), 10);

	  motor[motorFrontLeft] = normalize10(y2 - x2) * JoyToWheel;
	  motor[motorFrontRight] = normalize10(y1 + x1) * JoyToWheel;
	  motor[motorBackLeft] = normalize10(y2 + x1) * JoyToWheel;
	  motor[motorBackRight] = normalize10(y1 - x2) * JoyToWheel;
  }
}
