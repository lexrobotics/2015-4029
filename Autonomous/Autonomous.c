#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  HTMotor)
#pragma config(Sensor, S2,     backUltra,      sensorSONAR)
#pragma config(Sensor, S3,     frontUltra,     sensorSONAR)
#pragma config(Motor,  mtr_S1_C1_1,     liftMotors,    tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     leftMotors,    tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     rightMotors,   tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_2,     scoop,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_1,     tilt,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_2,     nomotor,       tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C3_1,    grabber,              tServoStandard)
#pragma config(Servo,  srvo_S1_C3_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_6,    servo6,               tServoNone)

#define AUTO_COMPETITION

#include "JoystickDriver.c"
#include "Autonomous.c"
#include "paths/Ramp.c"

/* Autonomous.c
   Final layer of abstraction
*/

task main() {
	waitForStart();
	Ramp();
}
