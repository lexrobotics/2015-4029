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
#pragma config(Servo,  srvo_S2_C1_1,    ballRamp,             tServoStandard)
#pragma config(Servo,  srvo_S2_C1_2,    egLift,               tServoStandard)
#pragma config(Servo,  srvo_S2_C1_3,    egRelease,            tServoStandard)
#pragma config(Servo,  srvo_S2_C1_4,    clamp2,               tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C1_5,    frontTurret,          tServoStandard)
#pragma config(Servo,  srvo_S2_C1_6,    servo6,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_1,    clamp1,               tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C2_2,    rearTurret,           tServoStandard)
#pragma config(Servo,  srvo_S2_C2_3,    kickstand,            tServoStandard)
#pragma config(Servo,  srvo_S2_C2_4,    grabberLift1,         tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C2_5,    grabber,              tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C2_6,    grabberLift2,         tServoContinuousRotation)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "../../Common/Ultrasonic-SMUX.h"
//#include "../../Common/IMU.h"
#include "../../Common/Movement.h"

void Position1() {
	sound(1,0);
	turnUltra(0, 0);
	turnUltra(1, 0);
	pause(0.1);
	moveDistance(50, 18);
	deployKnocker();
	pause(0.5);
	turnDistance(50, 120);
	//pause(0.5);
	//tillSense(-50, 0, true, 100, frontUS);
 //	moveDistance(100, 60);
}

void performTheCharge() {
	tillSense(200, 0, false, 30, frontUS);
	if(USreadDist(frontUS) > 7) {
		pause(0.1);
		moveDistance(-100, 10);
		pause(0.1);
		translateDistance(-200, 90, 12);
		pause(0.1);
		performTheCharge();
	}
	else {
		translateDistance(-200, 90, 7);
	}
}

void Position2() {
	sound(2,0.2);
	turnUltra(0, 0);
	turnUltra(1, 0);
	pause(0.1);
	turnDistance(50, 40);
	tillSense(200, 0, false, 60, rearUS);
	deployKnocker();
	tillSense(150, 270, false, 25, rearUS);
	pause(0.5);
	moveDistance(30,20);
	}

void Position3() {
	sound(3,0.5);
	translateDistance(200, 90, 40);
	pause(0.2);
	turnUltra(0, 0);
	pause(0.2);
	tillSense(50, 0, false, 90, rearUS);
	tillSense(200, 270, false, 30, rearUS);
	deployKnocker();
	pause(0.5);
	moveDistance(30, 20);

}


void Kickstand() {
	retractKnocker();
	turnUltra(0, 90);
	moveDistance(50, 20);
	pause(0.3);
	int position = detectPosition();
	switch(position) {
		case 1:
			nxtDisplayCenteredTextLine(2, "POSITION 1");
			Position1();
			break;
		case 2:
			nxtDisplayCenteredTextLine(2, "POSITION 2");
			Position2();
			break;
		case 3:
			nxtDisplayCenteredTextLine(2, "POSITION 3");
			Position3();
			break;
		default:
			return;
	}
	moveDistance(100,10);
}

#ifndef AUTO_COMPETITION
#ifndef RAMP_KICKSTAND
task main() {
	Kickstand();

}
#endif
#endif
