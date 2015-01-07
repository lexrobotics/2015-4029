#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S3,     ultra1,         sensorSONAR)
#pragma config(Sensor, S4,     ultra0,         sensorSONAR)
#pragma config(Motor,  motorB,           ,             tmotorNXT, PIDControl, reversed, encoder)
#pragma config(Motor,  mtr_S1_C1_1,     motorFrontLeft, tmotorTetrix, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     motorBackLeft, tmotorTetrix, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     motorBackRight, tmotorTetrix, PIDControl, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     motorFrontRight, tmotorTetrix, PIDControl, reversed, encoder)
#pragma config(Servo,  srvo_S1_C3_1,    servo1,               tServoStandard)
#pragma config(Servo,  srvo_S1_C3_2,    servo2,               tServoStandard)
#pragma config(Servo,  srvo_S1_C3_3,    grabber,              tServoContinuousRotation)
#pragma config(Servo,  srvo_S1_C3_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "JoystickDriver.c"
#include "drivers/hitechnic-irseeker-v2.h"
#include "../../Common/Movement.h"
#define robotLength 12.0

void Position1() {
	turnUltra(0, 90);
	tillSense(-50, 0, false, 20, ultra0);
	turnDistance(-50, 90);
	turnUltra(1, 0);
	pause(0.1);
	tillSense(50, 0, true, 25, ultra1);
}

void Position2() {
	turnUltra(0, -10);
	turnUltra(1, -10);
	pause(0.1);
	moveDistance(-50, 12);
	turnDistance(-50, 45);
	tillSense(-50, 0, false, 90, ultra1);
	pause(0.2);
	parallel(30, 0, ultra0, ultra1);
}

void CenterGoal() {
	turnUltra(0, 100);
	moveDistance(-50, 20);
	pause(0.3);
	float avg = 0;
	const int READINGS = 30;
	for(int i=0; i<READINGS; i++) {
		avg += SensorValue[ultra0];
		wait1Msec(20):
	}
	avg /= READINGS;
	int position;
	if(avg < 60)
		position = 1;
	else if(110 > avg && avg > 65)
		position = 3;
	else
		position = 2;

	switch(position) {
		case 1:
			Position1();
		case 2:
			Position2();
		default:
			return;
	}
}

#ifndef AUTO_COMPETITION
task main() {
	CenterGoal();
}
#endif
