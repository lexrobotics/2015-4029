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
#pragma config(Servo,  srvo_S2_C1_1,    no,                   tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C1_2,    ballDrop,             tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C1_3,    centerLift,           tServoContinuousRotation)
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

#include "JoystickDriver.c"
#include "drivers/hitechnic-irseeker-v2.h"
#include "../../Common/Movement.h"
#define robotLength 12.0

task main()
{
	//turnUltra(0, -20);
	//turnUltra(1,-20);
	while(true){
		nxtDisplayCenteredTextLine(1,"front: %d", SensorValue[frontUltra]);
		nxtDisplayCenteredTextLine(2,"back: %d", SensorValue[rearUltra]);
		wait1Msec(5);
		//		nxtDisplayCenteredTextLine(1,"front: %d", SensorValue[ultra1]);
		//nxtDisplayCenteredTextLine(2,"back: %d", SensorValue[ultra0]);
	}


}
