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

#define RAMP_KICKSTAND

#include "Kickstand.c"
#include "../../Common/Ultrasonic-SMUX.h"
#include "SixtyCM.h"
#include "../../Common/Movement.h"

void RampKickstand() {
	servo[kickstand] = 255;
	StartTask(init);
	turnUltra(0, 0);
	turnUltra(1, 0);
	releaseTube();

	SixtyCM();

	turnUltra(0, 0);
	pause(0.3);
	tillSense(200, 90, true, 90, rearUS);
	pause(0.1);
	turnWithGyro(-50, 120);
	pause(0.1);
	releaseTube();
	pause(0.2);
	turnWithGyro(-50, 60);
	grabTube();
	move(-50);
	pause(1);
	move(0);
	pause(0.5);
	Kickstand();
	//moveDistance(100, 5);
	//servo[lift1] = WINCHSTOP;
	//servo[lift2] = WINCHSTOP;
}

#ifndef AUTO_COMPETITION
task main() {
	RampKickstand();
}
#endif
