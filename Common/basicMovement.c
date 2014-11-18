#include "drivers\hitechnic-irseeker-v2.h"
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

void moveDistanceControlled(float speed, int distance) {
	float target = inchesToEncoder(distance);
	resetEncoders();
	float adjustedSpeed = speed;
	float error = distance;
	while(abs(error) > 5) {
		int error = target - abs(nMotorEncoder[leftMotors] + nMotorEncoder[rightMotors])/2;
		adjustedSpeed = speed;
		if(error < target) {
			 adjustedSpeed = (speed * (1.0 - abs(target-error)/target));
		}
		nxtDisplayCenteredTextLine(2, "s: %d, e: %d", adjustedSpeed, error);
		move(adjustedSpeed);
	}
}

void moveDistancePID(float speed, int distance) {
	static float kP = 0.02;
	float target = inchesToEncoder(distance);

	if(speed < 0)
		target *= -1;

	speed = abs(speed);

	resetEncoders();
	float error = target;
	while(abs(error) > 100) {
		nxtDisplayCenteredTextLine(2, "%d, %d", target, nMotorEncoder[rightMotors]);
		error = target - nMotorEncoder[rightMotors];
		float p = sgn(error) * 0 + kP * error;
		if(abs(p) > speed)
			move(sgn(p)*speed);
		else
			move(p);
	}
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
