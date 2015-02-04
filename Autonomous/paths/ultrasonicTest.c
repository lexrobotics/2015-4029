#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  none)
#pragma config(Hubs,  S2, HTServo,  HTServo,  none,     none)
#pragma config(Sensor, S3,     rearUltra,      sensorSONAR)
#pragma config(Sensor, S4,     frontUltra,     sensorSONAR)
#pragma config(Motor,  mtr_S1_C1_1,     motorFrontLeft, tmotorTetrix, PIDControl, reversed, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     motorBackLeft, tmotorTetrix, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     motorFrontRight, tmotorTetrix, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     motorBackRight, tmotorTetrix, PIDControl, reversed, encoder)
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
