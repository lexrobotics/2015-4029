#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  none)
#pragma config(Hubs,  S2, HTServo,  HTServo,  HTMotor,  none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ,               sensorI2CMuxController)
#pragma config(Sensor, S3,     HTSMUX,         sensorI2CCustom)
#pragma config(Sensor, S4,     HTSPB,          sensorI2CCustomFast9V)
#pragma config(Motor,  mtr_S1_C1_1,     motorBackRight, tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     motorFrontRight, tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     motorBackLeft, tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     motorFrontLeft, tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C3_1,     lift1,         tmotorTetrix, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C3_2,     harvester,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C3_1,     lift2,         tmotorTetrix, PIDControl, reversed, encoder)
#pragma config(Motor,  mtr_S2_C3_2,     conveyor,      tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S2_C1_1,    centerLift,           tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C1_2,    ballDrop,             tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C1_3,    sock,                 tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C1_4,    clamp1,               tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C1_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_6,    servo6,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_1,    frontTurret,          tServoStandard)
#pragma config(Servo,  srvo_S2_C2_2,    rearTurret,           tServoStandard)
#pragma config(Servo,  srvo_S2_C2_3,    kickstand,            tServoStandard)
#pragma config(Servo,  srvo_S2_C2_4,    grabber,              tServoStandard)
#pragma config(Servo,  srvo_S2_C2_5,    grabberLift1,         tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C2_6,    grabberLift2,         tServoContinuousRotation)
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
#include "../common/Util.h"

bool slow = false;
int reverse = 1;

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


void initTeleOp() {
	servo[kickstand] = 255;
	servo[clamp1] = 127;
	servo[grabberLift1] = 127;
	servo[grabberLift2] = 127;
}

task arm() {
  getJoystickSettings(joystick);
	while(true) {
		if(joystick.joy2_TopHat == 0) {
			motor[conveyor] = 100;
		}
		else if(joystick.joy2_TopHat == 4) {
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
			motor[harvester] = 100;
		}
		else if(joy1Btn(8)) {
			motor[harvester] = -100;
		}
		else {
			motor[harvester] = 0;
		}

		if(joy1Btn(2)) {
			servo[grabberLift1] = 255;
			servo[grabberLift2] = 0;
		}
		else if(joy1Btn(4)) {
			servo[grabberLift1] = 0;
			servo[grabberLift2] = 255;
		}
		else {
			servo[grabberLift1] = 127;
			servo[grabberLift2] = 127;
		}

		//if(joy2Btn(2)) {
		//	servo[sock] = 255;
		//}
		//else if(joy2Btn(4)) {
		//	servo[sock] = 0;
		//}
		//else {
		//	servo[sock] = 127;
		//}

		if(joy2Btn(5)) {
			motor[lift1] = 100;
		}
		else if(joy2Btn(7)) {
			motor[lift1] = -100;
		}
		else {
			motor[lift1] = 0;
		}

		if(joy2Btn(6)) {
			motor[lift2] = 100;
		}
		else if(joy2Btn(8)) {
			motor[lift2] = -100;
		}
		else {
			motor[lift2] = 0;
		}

		if(joy2Btn(3)) {
			servo[kickstand] = 140;
		}
		else {
			servo[kickstand] = 255;
		}

		if(joy1Btn(9)) {
			while(joy1Btn(9));
			slow = !slow;
			PlaySound(soundBeepBeep);
		}
		if(joy1Btn(10)) {
			while(joy1Btn(10));
			reverse = -1 * reverse;
			PlaySound(soundBeepBeep);
		}
	}
}

task main(){
  float x1, y1, x2, y2;
  bool harvesting = false;
  bool belting = false;

	waitForStart();
	initTeleOp();
	StartTask(arm);
  while(true){
  	getJoystickSettings(joystick);

  	//AUGMENTED-TANK JOYSTICKING SYSTEM
	  x1 = normalize10(reverse * joystick.joy1_x1);
	  x2 = normalize10(reverse * joystick.joy1_x2);
	  y1 = normalize10(reverse * joystick.joy1_y1);
	  y2 = normalize10(reverse * joystick.joy1_y2);

	  if(slow) {
	  	x1 *= 0.2;
	  	x2 *= 0.2;
	  	y1 *= 0.2;
	  	y2 *= 0.2;
	  }

	  //float JoyToWheel = 95.0 / max(max(max(abs(y2 + x2),abs(y1 - x1)),max(abs(y2 - x1),abs(y2 + x2))), 10);
		float joyToWheel = 1.0;
		if(reverse == -1) {
			motor[motorFrontRight] =  normalize10(y1 - x2) * joyToWheel;
		  motor[motorFrontLeft] = normalize10(y2 + x1) * joyToWheel;
		  motor[motorBackRight] =  normalize10(y1 + x1) * joyToWheel;
		  motor[motorBackLeft] =  normalize10(y2 - x2) * joyToWheel;
		}
		else {
		  motor[motorFrontLeft] =  normalize10(y1 + x2) * joyToWheel;
		  motor[motorFrontRight] = normalize10(y2 - x1) * joyToWheel;
		  motor[motorBackLeft] =  normalize10(y1 - x1) * joyToWheel;
		  motor[motorBackRight] = normalize10(y2 + x1) * joyToWheel;
		}
  }
}
