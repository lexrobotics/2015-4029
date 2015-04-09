#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  none)
#pragma config(Hubs,  S2, HTServo,  HTServo,  HTMotor,  none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ,               sensorI2CMuxController)
#pragma config(Sensor, S3,     HTSMUX,         sensorI2CCustom)
#pragma config(Sensor, S4,     HTSPB,          sensorI2CCustomFast9V)
#pragma config(Motor,  mtr_S1_C1_1,     motorBackRight, tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     motorFrontRight, tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     motorBackLeft, tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     motorFrontLeft, tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C3_1,     lift1,         tmotorTetrix, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C3_2,     harvester,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C3_1,     lift2,         tmotorTetrix, PIDControl, reversed, encoder)
#pragma config(Motor,  mtr_S2_C3_2,     conveyor,      tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S2_C1_1,    no,                   tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C1_2,    ballDrop,             tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C1_3,    centerLift,           tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C1_4,    clamp2,               tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C1_5,    frontTurret,         tServoStandard)
#pragma config(Servo,  srvo_S2_C1_6,    servo6,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_1,    clamp1,               tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C2_2,    rearTurret,           tServoStandard)
#pragma config(Servo,  srvo_S2_C2_3,    kickstand,            tServoStandard)
#pragma config(Servo,  srvo_S2_C2_4,    grabber,              tServoStandard)
#pragma config(Servo,  srvo_S2_C2_5,    servo11,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_6,    grabberLift2,         tServoContinuousRotation)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//#include "JoystickDriver.c"
//#include "drivers/hitechnic-irseeker-v2.h"
#include "JoystickDriver.c"
#include "../../Common/Ultrasonic-SMUX.h"
#include "../../Common/Touch.h"
#include "../../Common/Movement.h"
#include "../../Common/IMU.h"

bool everPressed = false;

void scoreBall() {
	servo[ballDrop] = 255;
	pause(1.6);
	servo[ballDrop] = 127;
}

void CenterPosition1() {
	turnUltra(0, 90);
	moveDistanceRamp(50, 12);
	binaryTillSenseHeading(80, 270, 30, frontUS);
	PlaySound(soundLowBuzzShort);
	translateDistance(200, 270, 7);
	writeDebugStream("DONE WITH FIRST");
	//binaryTillSense(35, 0, 30, clampUS);
	turnUltra(0,0);
	pause(0.2);
	//repeatedTillSense(35, 0, false, 60, clampUS);
	//binaryTillSense(35, 0, 20, clampUS);
	irTillSensePeak(30);
	playSound(soundLowBuzz);
	//turnUltra(0,40);
	//pause(0.2);
	//if(USreadDist(frontUS)<30){
	//	binaryTillSenseHeading(30, 0, 10, frontUS);
	//}
	turnWithGyro(30,0);
}

void CenterPosition2() {
	//turnDistance(-100, 50);
	turnWithGyro(70, -55);
	translateDistanceHeading(100, 90, 4);
	turnUltra(0, 0);
	pause(0.2);
	irTillSensePeak(30);
		playSound(soundLowBuzz);
	//turnUltra(0,20);
	//pause(0.2);
	//if(USreadDist(frontUS)<40){
	//	binaryTillSenseHeading(20, 0, 10, frontUS);
	//}
	turnWithGyro(30,0);
}

void CenterPosition3() {
	turnUltra(0, 0);
	turnUltra(1, 0);
	turnWithGyro(50, -90);
	//pause(0.2);
	//incrementalParallel(25, 2, rearUS, frontUS);
	//pause(0.2);
	//translateDistance(100, 90, 16);
	//pause(0.2);
		//repeatedTillSense(40,0,false, 40, clampUS);
	////binaryTillSense(40,0, 30, clampUS);
	//pause(0.5);
	//moveDistanceRamp(-50, 3);
	//pause(0.2);
}

void CenterToKickstand(int position) {
	translateDistance(100,270,10);
	pause(0.2);
	moveDistance(-100, 25);
	pause(0.2);
	turnDistance(50,110);

	turnUltra(0,0);
	pause(0.2);
	repeatedTillSense(50, 0, false, 90, frontUS);
	pause(0.2);
	if(position == 2)
		tillSense(200, 270, false, 27, frontUS);
	else if(position == 1)
		tillSense(200, 270, false, 22, frontUS);
	else if(position == 3);
		tillSense(200, 270, false, 25, frontUS);

	pause(0.2);
	deployKnocker();
	moveDistance(30, 10);
	moveDistance(100, 30);

}

task ohno() {
	while(true) {
		if(!everPressed && time1[T4] > 25000) {
			StopAllTasks();
		}
	}
}

int CenterGoal() {
	ClearTimer(T4);
	//StartTask(ohno);
	retractKnocker();
	turnUltra(0, 90);
	//moveDistance(50, 20);
	//StartTask(raiseLift);
	moveDistanceRamp(40, 20);
	//translateRTHeading(40);
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
			return -1;
	}

	//while(!lifted);
	deployClamp();

	readAllSwitches();
	//scoring commences
	ClearTimer(T1);
	int time = 35;
	bool anything_pressed = false;
	while(!sideSwitch || !armSwitch) {
		readAllSwitches();
		//if(sideSwitch) { PlaySound(soundLowBuzz); anything_pressed = true; everPressed = true; }
		//if(armSwitch) { PlaySound(soundUpwardTones); anything_pressed = true; everPressed = true;}
		if(tipSwitch) {
			move(-20);
		}
		else if(sideSwitch) {
			move(20);
		}
		else {
			translateRT(100, 90);
		}
	}
	PlaySound(soundBeepBeep);
	// hold position to score
	//while(true) {
	//	readAllSwitches();
	//	//if(sideSwitch) { PlaySound(soundLowBuzz); anything_pressed = true; everPressed = true; }
	//	//if(armSwitch) { PlaySound(soundUpwardTones); anything_pressed = true; everPressed = true;}
	//	if(!armSwitch) {
	//		move(20);
	//	}
	//	else {
	//		translateRT(100, 80);
	//	}
	//}
	//scoreBall();
	//move(0);
	return position;
}

#ifndef AUTO_COMPETITION
task main() {
	servo[clamp2] = 127;
	servo[clamp1] = 127;

	resetArduino();
	pause(5);
	PlaySound(soundBeepBeep);
	//int position = CenterGoal();
	translateXYAccel(0,0);
	while(true);
	//irTillSensePeak(30);
	//CenterToKickstand(position);
}
#endif
