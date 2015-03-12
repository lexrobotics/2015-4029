#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  none)
#pragma config(Hubs,  S2, HTServo,  HTServo,  HTMotor,  none)
#pragma config(Sensor, S3,     HTSMUX,         sensorI2CCustom)
#pragma config(Sensor, S4,     HTSPB,          sensorI2CCustom9V)
#pragma config(Motor,  mtr_S1_C1_1,     motorBackRight, tmotorTetrix, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     motorFrontRight, tmotorTetrix, PIDControl, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     motorBackLeft, tmotorTetrix, PIDControl, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     motorFrontLeft, tmotorTetrix, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C3_1,     motorH,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     harvester,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C3_1,     motorj,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C3_2,     conveyor,      tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S2_C1_1,    clamp1,               tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C1_2,    servo2,               tServoNone)
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

//#include "JoystickDriver.c"
//#include "drivers/hitechnic-irseeker-v2.h"
#include "../../Common/Ultrasonic-SMUX.h"
#include "../../Common/Touch.h"
#include "../../Common/Movement.h"

#define robotLength 12.0

void sound(int number, float pausetime){
	int i;
	for(i=0; i<number;i++){
		playsound(soundBlip);
		pause(pausetime);
	}
}

void deployClamp() {
	servo[clamp1] = 0;
	pause(1.5);
	servo[clamp1] = 127;
}

void Position1() {
	translateDistance(200, 270, 30);
	tillSense(100,0,false, 50, clampUS);
	pause(0.5);
	moveDistance(-50, 3);
}

void Position2() {
	turnDistance(-100, 35);
	tillSense(100,0,false, 50, clampUS);
	pause(0.5);
	moveDistance(-50, 5);
}

void Position3() {
	turnUltra(0, 0);
	turnUltra(1, 0);
	moveDistance(50, 5);
	turnDistance(-100, 95);
	pause(0.2);
	incrementalParallel(25, 2, rearUS, frontUS);
	pause(0.2);
	while(USreadDist(clampUS) > 40) {
		tillSense(50,0,false, 40, clampUS);
	}
	pause(0.5);
}

void CenterGoal() {
	turnUltra(0, 90);
	moveDistance(50, 20);
	pause(0.3);
	int position = detectPosition();
	switch(position) {
		case 1:
			sound(1, 0.2);
			Position1();
			break;
		case 2:
			sound(2, 0.2);
			Position2();
			break;
		case 3:
			sound(3, 0.2);
			Position3();
			break;
		default:
			return;
	}

	deployClamp();

	readAllSwitches();
	while(!sideSwitch || !armSwitch) {
		readAllSwitches();
		if(sideSwitch) { PlaySound(soundLowBuzz); }
		if(armSwitch) { PlaySound(soundUpwardTones) }
		if(sideSwitch) {
			move(-20);
		}
		else {
			translateRT(100, 90);
		}
	}
}

#ifndef AUTO_COMPETITION
task main() {
	CenterGoal();
}
#endif
