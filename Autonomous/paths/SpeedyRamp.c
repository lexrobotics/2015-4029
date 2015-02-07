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

void SpeedyRamp() {
	StartTask(init);
	moveDistance(-100, 85);
	grabTube();
	moveDistance(100, 20);
	pause(0.1);

	tillSense(200, 90, true, 75, frontUltra);
	pause(0.1);
	//parallel(25, 2, rearUltra, frontUltra);
	pause(0.2);
	turnUltra(0, 90);
	pause(0.3);
	tillSense(100, 0, false, 60, frontUltra);
	pause(0.1);
	turnDistance(-100, 180);
	turnUltra(0, 0);
	pause(0.3);
	translateDistance(-200, 90, 24);
	pause(0.1);
	StartTask(releaseTube);
	pause(0.2);
	moveDistance(100, 5);
	servo[lift1] = WINCHSTOP;
	servo[lift2] = WINCHSTOP;
}

#ifndef AUTO_COMPETITION
task main() {
	SpeedyRamp();
}
#endif
