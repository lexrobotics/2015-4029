#include "drivers\hitechnic-irseeker-v2.h"
#include "..\common\PID.h"
const float DIAMETER = 4.0; //diameter of wheel in inches
const float ENCODER_SCALE = 1440.0; //number of encoder counts per rotation
const float CIRCUMFERENCE = DIAMETER * PI;
const float TURN_RADIUS = 11.02; //center of robot to turning circle in inches
const float TURN_CIRCUMFERENCE = 2.0 * TURN_RADIUS * PI; //circumference of circle robot turns in
const float TURN_SCALAR = 1.2; //because it's not a square

//int getIRSensor(){
//	const tMUXSensor HTIRS2 = msensor_S2_1;
//	int sector = 0;
//  tHTIRS2DSPMode _mode = DSP_1200;
//	sector = HTIRS2readACDir(HTIRS2);
//	return sensor;
//}
void resetEncoders()
{

	nMotorEncoder[leftMotors] = 0;
	nMotorEncoder[rightMotors] = 0;
	/*
	nMotorPIDSpeedCtrl[motorsRight] = mtrSpeedReg; //enable PID on all the motors
	nMotorPIDSpeedCtrl[motorsLeft] = mtrSpeedReg;
	nMotorPIDSpeedCtrl[motorRightFront] = mtrSpeedReg;
	nMotorPIDSpeedCtrl[motorRightBack] = mtrSpeedReg;
	*/
}

int inchesToEncoder(int distance) {
	 return distance/CIRCUMFERENCE * ENCODER_SCALE;
}

int degreesToEncoder(int angle) {
	return (angle/360.0 * TURN_CIRCUMFERENCE)/CIRCUMFERENCE * ENCODER_SCALE * TURN_SCALAR;
}

void move(int speed) {
	motor[leftMotors] = speed;
	motor[rightMotors] = speed;
}

void turn(int speed) {
	motor[leftMotors] = speed;
	motor[rightMotors] = -1 * speed;
}

void moveDistance(int speed, int distance) {
	int target = inchesToEncoder(distance);
	resetEncoders();

	while(abs(nMotorEncoder[leftMotors]) < abs(target)  //wait until position reached
		&& abs(nMotorEncoder[rightMotors]) < abs(target)) {
			move(speed); //move at desired speed
		}

	move(0); //stop
}

void moveDistancePID(int distance) {
	PID forwardsPID;
 	forwardsPID.Kp = 0.04;
 	forwardsPID.Ki = 0.00001;
 	forwardsPID.Kd = 0;
 	forwardsPID.integral = 0;

	float target = inchesToEncoder(distance);
	float error = target;
	float speed = 100;

	resetEncoders();
	while(abs(error) > 50) {
		error = target - nMotorEncoder[rightMotors];
		speed = updatePID(forwardsPID, error, nMotorEncoder[rightMotors]);
		writeDebugStreamLine("%f", speed);
		move(speed);
	}
	move(0);
}
void moveDistancePID(int distance, float _Kp, float _Ki) {
	PID forwardsPID;
 	forwardsPID.Kp = _Kp;
 	forwardsPID.Ki = _Ki;
 	forwardsPID.Kd = 0;
 	forwardsPID.integral = 0;

	float target = inchesToEncoder(distance);
	float error = target;
	float speed = 100;

	resetEncoders();
	while(abs(error) > 50) {
		error = target - nMotorEncoder[rightMotors];
		speed = updatePID(forwardsPID, error, nMotorEncoder[rightMotors]);
		writeDebugStreamLine("%f", speed);
		move(speed);
	}
	move(0);
}

void turnDistancePID(int angle) {
	PID turnPID;
 	turnPID.Kp = 0.1;
 	turnPID.Ki = 0.0001;
 	turnPID.Kd = 0;
 	turnPID.integral = 0;

	float target = -degreesToEncoder(angle);
	float error = target;
	float speed = 100;

	resetEncoders();
	while(abs(error) > 25) {
		error = target - nMotorEncoder[rightMotors];
		speed = updatePID(turnPID, error, nMotorEncoder[rightMotors]);
		writeDebugStreamLine("%f", speed);
		if(abs(speed) > 100)
			speed = sgn(speed) * 100;
		turn(-speed);
	}
	move(0);
}

void turnDistance(int speed, int angle) {
	int target = degreesToEncoder(angle);
	resetEncoders();

	while(abs(nMotorEncoder[leftMotors]) < abs(target)  //wait until position reached
		&& abs(nMotorEncoder[rightMotors]) < abs(target)) turn(speed); //turn at desired speed

	turn(0); //stop
}

void pause(float seconds) {
	wait1Msec(seconds * 1000);
}
