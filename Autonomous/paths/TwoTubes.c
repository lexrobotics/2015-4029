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

void TwoTubes() {
	StartTask(init); // Initialize all servos and stuff!
	turnUltra(0, 0); // Initiate ultrasonics to point at wall
	turnUltra(1, 0);

	StartTask(releaseTubeTask); // ready to receive tube
	StartTask(lowerGrabber);
	moveDistanceCoast(-100, 20);
	initialHeading = 0;
	translateDistanceHeading(-100, 0, 60);

	pause(0.1);
	moveDistancePID(-20);
	pause(0.1);
	//incrementalParallel(25, 2, rearUS, clampUS); //Parallel to the wall
	//pause(0.2);
	//tillSense(100, 270, 4, true, frontUS); //???
	//incrementalParallel(25, 2, rearUS, frontUS); //Parallel again to be extra sure
	//pause(0.2);
	turnUltra(1,0);
	pause(0.2);
	//moveDistancePID(-40); // move further to shift to the tube into the pentagon slot
	grabTube(); //Lower the tube grabber
	while(USreadDist(rearUS) == 255) {
		translateRTHeading(120, 90);
		pause(0.1);
	}
	translating = false;
	repeatedTillSenseHeading(120, 90, true, 65, clampUS);
	turnWithGyro(-100, 180);
	StartTask(lowerGrabber);
	pause(0.2);
	releaseTube(); pause(0.1); releaseTube();
	moveDistancePID(13);
	turnWithGyro(-100, 180);

	repeatedTillSenseHeading(-120, 90, false, 20, clampUS);

	translateDistanceHeading(-100,0,25);
	grabTube();
	moveDistance(100, 16);
	while(USreadDist(rearUS) == 255) {
		translateRTHeading(120, 90);
		pause(0.1);
	}
	translating = false;
	pause(0.1);
	repeatedTillSenseHeading(120, 90, true, 65, clampUS);
	pause(0.1);
	pause(0.2);
	turnUltra(0, 85);
	pause(0.3);
	initialHeading=-10;
	translateDistance(100, 0, 24);

	repeatedTillSenseHeading(100, 0, false, 30, frontUS);
	pause(0.2);
	repeatedTillSenseHeading(120, 90, true, 60, clampUS);
	moveDistance(-50, 5);
	turnWithGyro(-100, 140);
	releaseTube();releaseTube();
	moveDistance(100, 15);
}

#ifndef AUTO_COMPETITION
task main() {
	resetArduino();
	pause(5);
	PlaySound(soundBeepBeep);
	TwoTubes();
}
#endif
