#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  none)
#pragma config(Hubs,  S2, HTServo,  HTServo,  HTMotor,  none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ,               sensorI2CMuxController)
#pragma config(Sensor, S3,     rearUltra,      sensorSONAR)
#pragma config(Sensor, S4,     frontUltra,     sensorSONAR)
#pragma config(Motor,  mtr_S1_C1_1,     motorFrontLeft, tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     motorBackLeft, tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     motorFrontRight, tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     motorBackRight, tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C3_1,     conveyor,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     harvester,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C3_1,     motorj,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C3_2,     tubeLift,      tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S2_C1_1,    lift1,                tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C1_2,    lift2,                tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C1_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_6,    servo6,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_1,    frontTurret,          tServoStandard)
#pragma config(Servo,  srvo_S2_C2_2,    rearTurret,           tServoStandard)
#pragma config(Servo,  srvo_S2_C2_3,    grabber,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_4,    servo10,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_5,    servo5,               tServoStandard)
#pragma config(Servo,  srvo_S2_C2_6,    servo6,               tServoStandard)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "JoystickDriver.c"
#include "drivers/hitechnic-irseeker-v2.h"
#include "../../Common/Movement.h"
#define robotLength 12.0

bool ultraReady = true;
int ultraAngle = 0;

void grabTube(){
	servo[grabber] = 255;
	pause(0.7);
	servo[grabber] = 127;
}
task releaseTube(){
	servo[grabber] = 0;
	pause(0.9);
	servo[grabber] = 127;
}

task init() {
	startTask(releaseTube);
	//turnUltra(0);
}

/*
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
	servo[bucketTilt] = 245;
	pause(1);
	servo[bucketGate] = 105;
	pause(0.8);
	servo[bucketGate] = 10;
	pause(0.5);
	servo[bucketTilt] = 80;
	pause(0.5);
	ClearTimer(T3);
	while(abs(nMotorEncoder[liftStageTwo]) > 0.5 * 1440 && time1[T3] < 1500) {
		nxtDisplayCenteredTextLine(2, "%d", nMotorEncoder[liftStageOne]);
		if(abs(nMotorEncoder[liftStageTwo]) > 0.5 * 1440)
			motor[liftStageTwo] = 100;
		else
			motor[liftStageTwo] = 0;
	}
	motor[liftStageTwo] = 0;
	servo[bucketTilt] = 120;
	ClearTimer(T3);
	while(abs(nMotorEncoder[liftStageTwo]) > 0.4 * 1440 && time1[T3] < 1000) {
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
*/


void RampKickstand(){
	// Navigate down the ramp and grab tube
	pause(0.5);
	startTask(init);
	moveDistancePID(-83, 0.02, 0);
	//StartTask(scoreAutoBall);
	grabTube();
	// bring tube to goal
	turnDistance(50, 20);
	moveDistance(50, 10);
	turnDistance(50, 10);
	moveDistance(50, 60);
	turnDistance(50, 150);
	resetEncoders();
	turnUltra(0, 90);
	while(SensorValue[ultra0] > 65) {
		move(-100);
	}
	startTask(releaseTube);
	//parallel and get in front of ramp

	turnDistance(50, 230);

	//angle towards the wall and turn the ultra perpendicular to the wall
	//changed for speed
	moveDistance(-50, 75);
	turnDistance(-50, 35);

	pause(0.2);
	turnUltra(0, 0);
	turnUltra(1, 0);
	pause(0.3);


	parallel(20, 0, ultra0, ultra1);
	pause(0.2);
	tillSense(-50, 90, false, 22, ultra1);
	pause(0.5);

	turnUltra(0, 120);
	pause(0.75);
	tillSense(-50, 0, false, 20, ultra0);

	grabTube();
	turnUltra(0,0);
	parallel(20, 0, ultra0, ultra1);
	pause(0.2);
	turnDistance(50, 15);
	moveDistance(50, 94);
	turnDistance(30, 150);

	resetEncoders();
	turnUltra(0, 90);
	while(SensorValue[ultra0] > 65) {
		move(-100);
	}
	StartTask(releaseTube);
}

#ifndef AUTO_COMPETITION
task main() {
	RampKickstand();
	//translateDistance(100, 45, 24);
}
#endif
