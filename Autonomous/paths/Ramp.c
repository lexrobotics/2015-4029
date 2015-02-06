#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  none)
#pragma config(Hubs,  S2, HTServo,  HTServo,  none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ,               sensorI2CMuxController)
#pragma config(Sensor, S3,     rearUltra,      sensorSONAR)
#pragma config(Sensor, S4,     frontUltra,     sensorSONAR)
#pragma config(Motor,  mtr_S1_C1_1,     motorFrontRight, tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     motorBackRight, tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     motorFrontLeft, tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     motorBackLeft, tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C3_1,     conveyor,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     harvester,     tmotorTetrix, openLoop)
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

const int WINCHSTOP = 12;

void grabTube(){
	servo[grabber] = 255;
	pause(0.7);
	servo[grabber] = 127;
}
task releaseTube(){
	servo[grabber] = 0;
	pause(1.1);
	servo[grabber] = 127;
}

task init() {
	startTask(releaseTube);
	turnUltra(0, 0);
	turnUltra(1, 0);
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


void Ramp(){
	// Navigate down the ramp and grab tube
	pause(0.5);
	startTask(init);
	//moveDistancePID(-83, 0.02, 0);
	moveDistance(-50, 65);
	pause(0.5);
	tillSense(200, 90, true, 70, frontUltra);
	pause(0.1);
	parallel(25, 2, rearUltra, frontUltra);
	pause(0.1);
	moveDistance(-20,10);
	parallel(25, 1, rearUltra, frontUltra);
	pause(0.1);
	turnUltra(1, 70);
	tillSense(-100, 0, false, 65, rearUltra);
	while(SensorValue[rearUltra] > 30) {
		move((-20.0 * SensorValue[rearUltra]/100.0) - 10)
	}
	move(0);
	grabTube();
	turnUltra(1, 0);
	pause(0.5);
	moveDistance(100,40);
	pause(0.2);
	tillSense(200, 90, true, 70, frontUltra);
	pause(0.2);
	parallel(25, 2, rearUltra, frontUltra);
	pause(0.2);
	turnUltra(0, 85);
	pause(0.6);
	tillSense(100, 0, false, 55, frontUltra);
	pause(0.1);
	turnDistance(-100, 180);
	pause(0.1);
	translateDistance(-200, 90, 24);
	pause(0.1);
	turnUltra(0, 0);
	StartTask(releaseTube);
	pause(0.2);
	moveDistance(100, 5);
	translateDistance(200, 90, 24);
	turnDistance(100, 170);
	moveDistance(-100,40);
	pause(0.2);
	parallel(25, 2, rearUltra, frontUltra);
	pause(0.2);
	tillSense(200, 90, true, 70, frontUltra);
	turnUltra(1,90);
	pause(0.3);
	tillSense(-100, 0, false, 45, rearUltra);
	turnDistance(100, 90);
	moveDistance(-50, 24);
	grabTube();
}

#ifndef AUTO_COMPETITION
task main() {
	//turnUltra(0,0)
	//pause(0.8);
	//parallel(20, 1, rearUltra, frontUltra);
 	Ramp();
	//translateDistance(100,90,90);
	//turnDistance(-20,90);
	//turnDistance(50, 230);
	//moveDistance(50,100);
}
#endif
