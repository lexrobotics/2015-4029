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

//#include "JoystickDriver.c"
//#include "drivers/hitechnic-irseeker-v2.h"
#include "../../Common/Ultrasonic-SMUX.h"
#include "../../Common/Touch.h"
#include "../../Common/Movement.h"

void scoreBall() {
	servo[ballDrop] = 255;
	pause(1.6);
	servo[ballDrop] = 127;
}

void CenterPosition1() {
	turnUltra(0, 90);
	moveDistanceRamp(50, 12);
	binaryTillSense(80, 270, 10, frontUS);
	PlaySound(soundLowBuzzShort);
	translateDistance(200, 270, 25);
	writeDebugStream("DONE WITH FIRST");
	//binaryTillSense(40, 0, 10, clampUS);
	repeatedTillSense(50, 0, false, 60, clampUS);
	pause(0.5);
	moveDistanceRamp(-50, 7);
}

void CenterPosition2() {
	turnDistance(-100, 50);
	//tillSense(100,0,false, 30, clampUS);
	binaryTillSense(50,0,10,clampUS);
	pause(0.5);
	moveDistanceRamp(-50, 3);
}

void CenterPosition3() {
	turnUltra(0, 0);
	turnUltra(1, 0);
	turnDistance(-100, 95);
	pause(0.2);
	incrementalParallel(25, 2, rearUS, frontUS);
	pause(0.2);
	translateDistance(100, 90, 16);
	pause(0.2);
	binaryTillSense(40,0, 10, clampUS);
	pause(0.5);
	moveDistanceRamp(-50, 3);
	pause(0.2);
}

void CenterToKickstand() {
	translateDistance(100,270,10);
	pause(0.2);
	moveDistance(-100, 25);
	pause(0.2);
	turnDistance(50,110);

	turnUltra(0,0);
	pause(0.2);
	binaryTillSense(40,0,10, frontUS);
	pause(0.2);
	tillSense(200, 270, false, 25, frontUS);
	pause(0.2);
	deployKnocker();
	moveDistance(30, 10);
	moveDistance(100, 30);

}

void CenterGoal() {
	retractKnocker();
	turnUltra(0, 90);
	//moveDistance(50, 20);
	StartTask(raiseLift);
	moveDistanceRamp(50, 20);
	//moveDistancePID(20);
	pause(1);
	int position = detectPosition();
	switch(position) {
		case 1:
			sound(1, 0.2);
			CenterPosition1();
			break;
		case 2:
			sound(2, 0.2);
			CenterPosition2();
			break;
		case 3:
			sound(3, 0.2);
			CenterPosition3();
			break;
		default:
			return;
	}

	while(!lifted);
	deployClamp();

	readAllSwitches();
	//scoring commences
	while(!sideSwitch || !armSwitch) {
		readAllSwitches();
		if(sideSwitch) { PlaySound(soundLowBuzz); }
		if(armSwitch) { PlaySound(soundUpwardTones); }
		if(sideSwitch || armSwitch) {
			move(-20);
		}

		else {
			translateRT(100, 90);
		}
	}
	PlaySound(soundBeepBeep);
	translateRT(200, 120);
	scoreBall();
	move(0);
}

#ifndef AUTO_COMPETITION
task main() {
	CenterGoal();
}
#endif
