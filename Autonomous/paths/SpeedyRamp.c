#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  none)
#pragma config(Hubs,  S2, HTServo,  HTServo,  HTMotor,  none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ,               sensorI2CMuxController)
#pragma config(Sensor, S3,     HTSMUX,         sensorI2CCustom)
#pragma config(Sensor, S4,     HTSPB,          sensorI2CCustomFast9V)
#pragma config(Motor,  mtr_S1_C1_1,     motorBackRight, tmotorTetrix, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     motorFrontRight, tmotorTetrix, PIDControl, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     motorBackLeft, tmotorTetrix, PIDControl, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     motorFrontLeft, tmotorTetrix, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C3_1,     motorH,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     harvester,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C3_1,     motorj,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C3_2,     conveyor,      tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S2_C1_1,    lift1,                tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C1_2,    lift2,                tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C1_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_6,    servo6,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_1,    frontTurret,          tServoStandard)
#pragma config(Servo,  srvo_S2_C2_2,    rearTurret,           tServoStandard)
#pragma config(Servo,  srvo_S2_C2_3,    kickstand,            tServoStandard)
#pragma config(Servo,  srvo_S2_C2_4,    grabber,              tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C2_5,    servo5,               tServoStandard)
#pragma config(Servo,  srvo_S2_C2_6,    servo6,               tServoStandard)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "../../Common/Ultrasonic-SMUX.h"
//#include "../../Common/IMU.h"
#include "../../Common/Movement.h"

#define robotLength 12.0

void SpeedyRamp() {
	StartTask(init); // Initialize all servos and stuff!

	turnUltra(0, 0); // Initiate ultrasonics to point at wall
	turnUltra(1, 0);

	releaseTube(); // ready to receive tube

	moveDistancePID(-70); // move down the ramp

	pause(0.1);
	incrementalParallel(25, 2, rearUS, frontUS); //Parallel to the wall
	pause(0.2);
	tillSense(100, 270, 4, true, frontUS); //???
	incrementalParallel(25, 2, rearUS, frontUS); //Parallel again to be extra sure
	pause(0.2);

	moveDistancePID(-40); // move further to shift to the tube into the pentagon slot

	grabTube(); //Lower the tube grabber
	servo[grabber] = 0; //...?
	moveDistance(50, 16);
	servo[grabber] = 127;//.......
	pause(0.3);

	move(0);
	//tillSense(100, 0, false, 50, frontUS);
	//tillSense(100, 0, false, 60, frontUS);
	//pause(0.1);
	//turnDistance(-100, 180);
	//pause(0.3);
	//translateDistance(-200, 90, 40);
	turnUltra(0, 0);
	pause(0.3);
	tillSense(200, 90, true, 65, frontUS);
	pause(0.1);
	incrementalParallel(25, 2, rearUS, frontUS);
	pause(0.2);
	turnUltra(0, 90);
	pause(0.3);
	tillSense(100, 0, false, 50, frontUS);
	turnDistance(-50, 90);
	pause(0.1);
	releaseTube();
	pause(0.2);
	turnDistance(-50, 90);
	grabTube();
	move(-50);
	pause(1);
	move(0);
	//moveDistance(100, 5);
	//servo[lift1] = WINCHSTOP;
	//servo[lift2] = WINCHSTOP;
}

#ifndef AUTO_COMPETITION
task main() {
	SpeedyRamp();
}
#endif
