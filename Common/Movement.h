#ifndef MOVEMENT_H
#define MOVEMENT_H

#include "PID.h"
#include "Util.h"

/*
Movement constants
*/
const float DIAMETER = 4.0; //diameter of wheel in inches
const float ENCODER_SCALE = 1120.0; //number of encoder counts per rotation
const float CIRCUMFERENCE = DIAMETER * PI;
const float TURN_RADIUS = 12.73; //center of robot to turning circle in inches
const float TURN_CIRCUMFERENCE = 2.0 * TURN_RADIUS * PI; //circumference of circle robot turns in
const float TURN_SCALAR = 1.1; //because it's not a square

void fullStop();
void turn(int speed);
void turnWithGyro(int speed, int heading);
void translateRT(int speed, int angle);
void translateXY(int fwd, int right);
void resetEncoders();
int inchesToEncoder(int distance);
int degreesToEncoder(int angle);

const int WINCHSTOP = 12;

//void grabTube(){
//	servo[grabber] = 255;
//	pause(1);
//	servo[grabber] = 127;
//}
//task releaseTube(){
//	servo[grabber] = 0;
//	pause(1.3);
//	servo[grabber] = 127;
//}

task init() {
	//startTask(releaseTube);
}

/*
Motor business
*/
void fullStop(){
	motor[motorFrontLeft] = 0;
	motor[motorFrontRight] = 0;
	motor[motorBackLeft] = 0;
	motor[motorBackRight] = 0;
}

void move(int speed) {
	motor[motorFrontLeft] = speed;
	motor[motorBackRight] = speed;
	motor[motorFrontRight] = speed;
	motor[motorBackLeft] = speed;
}

void turn(int speed){
	motor[motorFrontLeft] = speed;
	motor[motorBackRight] = -speed;
	motor[motorFrontRight] = -speed;
	motor[motorBackLeft] = speed;
}

void moveDistance(int speed, int distance) {
	int target = inchesToEncoder(distance);
	resetEncoders();

	while(abs(nMotorEncoder[motorFrontLeft]) < abs(target)  //wait until position reached
		&& abs(nMotorEncoder[motorBackRight]) < abs(target)) {
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
		error = target - nMotorEncoder[motorBackRight];
		speed = updatePID(forwardsPID, error, nMotorEncoder[motorBackRight]);
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
		error = target - nMotorEncoder[motorBackRight];
		speed = updatePID(forwardsPID, error, nMotorEncoder[motorBackRight]);
		writeDebugStreamLine("%f", speed);
		if(abs(speed) < 20)
			speed = sgn(speed) * 20;
		move(speed);
	}
	move(0);
}

void translateDistance(int speed, int angle, int distance) {
	int target;
	if (angle % 90 == 0)
		target = inchesToEncoder(distance);
	else
		target = inchesToEncoder(distance * min(abs(1/cos(PI * angle / 180.0)), abs(1/sin(PI * angle / 180.0))));
	resetEncoders();

	while(abs(nMotorEncoder[motorFrontLeft]) < abs(target)  //wait until position reached
		&& abs(nMotorEncoder[motorBackRight]) < abs(target)
		&& abs(nMotorEncoder[motorFrontRight]) < abs(target)
		&& abs(nMotorEncoder[motorBackLeft]) < abs(target)) {
			translateRT(speed, angle); //move at desired speed
		}

	fullStop(); //stop
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
		error = target - nMotorEncoder[motorBackRight];
		speed = updatePID(turnPID, error, nMotorEncoder[motorBackRight]);
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

	while(abs(nMotorEncoder[motorFrontLeft]) < abs(target)  //wait until position reached
		&& abs(nMotorEncoder[motorBackRight]) < abs(target)) turn(speed); //turn at desired speed

	turn(0); //stop
}

void translateRT(int speed, int angle) {
	//if(speed > 60)
	//	speed = 60;
	float angleRad = (angle + 45) * PI/180;
	float pow1 = speed * sin(angleRad);
	float pow2 = speed * cos(angleRad);
	motor[motorFrontLeft] = pow1;
	motor[motorBackRight] = pow1;
	motor[motorFrontRight] = pow2;
	motor[motorBackLeft] = pow2;
}

void translateXY(int fwd, int right) {//It's a lot easier to use RT.
	motor[motorFrontLeft] = fwd + right;
	motor[motorBackRight] = fwd + right;
	motor[motorFrontRight] = fwd - right;
	motor[motorBackLeft] = fwd - right;
}

/*
Encoder business
*/
void resetEncoders() {
	nMotorEncoder[motorBackLeft] = 0;
	nMotorEncoder[motorBackRight] = 0;
	nMotorEncoder[motorFrontLeft] = 0;
	nMotorEncoder[motorFrontRight] = 0;
}

int inchesToEncoder(int distance) {
	 return distance/CIRCUMFERENCE * ENCODER_SCALE;
}

int degreesToEncoder(int angle) {
	return (angle/360.0 * TURN_CIRCUMFERENCE)/CIRCUMFERENCE * ENCODER_SCALE * TURN_SCALAR;
}

#endif /* MOVEMENT_H */
