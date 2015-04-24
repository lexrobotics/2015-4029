#ifndef MOVEMENT_H
#define MOVEMENT_H

#include "PID.h"
#include "Util.h"
#include "IMU.h"
/*
Movement constants
*/
const float DIAMETER = 4.0; //diameter of wheel in inches
const float ENCODER_SCALE = 1120.0; //number of encoder counts per rotation
const float CIRCUMFERENCE = DIAMETER * PI;
const float TURN_RADIUS = 12.73; //center of robot to turning circle in inches
const float TURN_CIRCUMFERENCE = 2.0 * TURN_RADIUS * PI; //circumference of circle robot turns in
const float TURN_SCALAR = 1.1; //because it's not a square

int translateSpeed;
int translateAngle;
bool translating = false;
float hugeBump = true;
float initialHeading = 0;

//Initialization
task init();

//Self-explanatory tube-grabber manipulation.
void releaseTube();
void grabTube();

//Basic tank-drive movement
void fullStop();
void move(int speed);
void turn(int speed);

//Movement abstractions - tank drive
void moveDistance(int speed, int distance);
void moveDistancePID(int distance);
void moveDistancePID(int distance, float _Kp, float _Ki);

//Movement abstractions - turning in place
void turnDistance(int speed, int angle);
void turnDistancePID(int angle);

//Movement abstraction - mecanum drive
void translateRT(int speed, int angle);
void translateXY(int fwd, int right);
void translateDistance(int speed, int angle, int distance);

//Encoder business for drive motors
void resetEncoders();
int inchesToEncoder(int distance);
int degreesToEncoder(int angle);

//Gyro-based turning
//void turnWithGyro(int speed, int heading);

float normalizeHeading(float angle) {
	return (angle + 360)%360;
}

task init() {
	//startTask(releaseTube);
	servo[egRelease] = 127;
	servo[kickstand] = 255; // kickstand hook
	servo[egLift] = 190;
	servo[grabberLift1] = 127;
	servo[grabberLift2] = 127;
}

bool lifted = false;
bool servosLifted = false;

task raiseServos() {
	servo[egLift] = 0;
	servosLifted = true;
}

task raiseGrabber() {
	servo[grabberLift1] = 255;
	servo[grabberLift2] = 0;
	pause(0.5);
	servo[grabberLift1] = 127;
	servo[grabberLift2] = 127;
}

task lowerGrabber() {
	pause(2);
	servo[grabberLift1] = 0;
	servo[grabberLift2] = 255;
	pause(1.6);
	servo[grabberLift1] = 127;
	servo[grabberLift2] = 127;
}

task raiseLift() {
	StartTask(raiseServos);
	servo[clamp1] = 0;
	pause(0.6);
	servo[clamp1] = 127;

	nMotorEncoder[lift1] = 0;
	nMotorEncoder[lift2] = 0;
	pause(0.3);
	int prevEncoder1 = -20;
	int prevEncoder2 = -20;
	const int ENCODER_TARGET = 8 * 1100;
	const int DIFF_THRESH = 10;

	while(abs(nMotorEncoder[lift1]) < ENCODER_TARGET ||
		abs(nMotorEncoder[lift2]) < ENCODER_TARGET) {
		motor[lift1] = -100;
		motor[lift2] = -100;
	}
	int diff1 = abs(nMotorEncoder[lift1] - prevEncoder1);
	int diff2 = abs(nMotorEncoder[lift2] - prevEncoder2);

	while(diff1 > DIFF_THRESH || diff2 > DIFF_THRESH) {
		diff1 = abs(nMotorEncoder[lift1] - prevEncoder1);
		diff2 = abs(nMotorEncoder[lift2] - prevEncoder2);
		motor[lift1] = -100;
		motor[lift2] = -100;
		prevEncoder1 = nMotorEncoder[lift1];
		prevEncoder2 = nMotorEncoder[lift2];
		nxtDisplayCenteredTextLine(2, "%d %d", nMotorEncoder[lift1], nMotorEncoder[lift2]);
		wait1Msec(100);
	}


	motor[lift1] = 0;
	motor[lift2] = 0;

	//while(!servosLifted);
	lifted = true;
}

task releaseTubeTask() {
	releaseTube();
	releaseTube();
}

void releaseTube() {
	servo[grabber] = 255; //.....................???
	pause(1);
	servo[grabber] = 127;
}

void grabTube() {
	servo[grabber] = 0;
	pause(1.4);
	servo[grabber] = 127;
}

void sound(int number, float pausetime){
	int i;
	for(i=0; i<number;i++){
		PlaySound(soundBlip);
		pause(pausetime);
	}
}

void deployClamp() {
	servo[clamp1] = 0;
	pause(1.9);
	servo[clamp1] = 127;
}

void deployKnocker() {
	servo[kickstand] = 70;
	pause(0.3);
	servo[kickstand] = 120;
}

void retractKnocker() {
	servo[kickstand] = 255;
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
void moveDistanceCoast(int speed, int distance) {
	int target = inchesToEncoder(distance);
	resetEncoders();

	while(abs(nMotorEncoder[motorFrontLeft]) < abs(target)  //wait until position reached
		&& abs(nMotorEncoder[motorBackRight]) < abs(target)) {
			move(speed); //move at desired speed
		}
 //stop
}

void moveDistancePID(int distance) {
	PID forwardsPID;
 	forwardsPID.Kp = 0.02;
	forwardsPID.Ki = 0.0000;
 	forwardsPID.Kd = 0;
 	forwardsPID.integral = 0;

	float target = inchesToEncoder(distance);
	float error = target;
	float speed = 50;
	int finalRamp = 40;
	resetEncoders();
	while(abs(error) > 50 && finalRamp > 15) {
		error = target - nMotorEncoder[motorBackRight];
		speed = updatePID(forwardsPID, error, nMotorEncoder[motorBackRight]);
		nxtDisplayCenteredTextLine(2, "%f", speed);
		if(abs(speed) < finalRamp) {
			speed = sgn(distance) * finalRamp;
			finalRamp--;
		}
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

//void turnDistanceGyro(float angle){
//	initialHeading = (initialHeading+ angle + 360) %360;
//	const float threshold = 2;
//	float error=initialHeading - normalizeHeading(getHeading()) ;
//	while(abs(error) - initialHeading > threshold){
//		error=initialHeading - normalizeHeading(getHeading()) ;
//		writeDebugStreamLine("error: %f", error);
//		turn((error/360.0) * 100);

//	}
//	turn(0);
//	//while(abs(normalizeHeading(getHeading()) - initialHeading) > threshold){
//	//	error = 360 - (initialHeading - normalizeHeading(getHeading()));
//	//	error /= 10;
//	//	float speed = (sgn(error)* -10) - error;
//	//	turn(speed);
//	//	writeDebugStreamLine("error: %f, speed: %f", error, speed);
//	//}
//	//turn(0);

////}


void turnDistance(int speed, int angle) {
	int target = degreesToEncoder(angle);
	resetEncoders();

	while(abs(nMotorEncoder[motorFrontLeft]) < abs(target)  //wait until position reached
		&& abs(nMotorEncoder[motorBackRight]) < abs(target)) turn(speed); //turn at desired speed

	turn(0); //stop
}

//void moveDistanceRamp(int speed, int distance) {
//	float target1 = 0.5 * inchesToEncoder(distance);
//	int target2 = inchesToEncoder(distance);
//	resetEncoders();

//	while(abs(nMotorEncoder[motorFrontLeft]) < abs(target1)  //wait until position reached
//		&& abs(nMotorEncoder[motorBackRight]) < abs(target1)) {
//			move(speed); //move at desired speed
//		}

//	int posFL = nMotorEncoder[motorFrontLeft];
//	int posBR = nMotorEncoder[motorBackRight];

//	while(abs(nMotorEncoder[motorFrontLeft]) < abs(target2)  //wait until position reached
//		&& abs(nMotorEncoder[motorBackRight]) < abs(target2)) {
//			move((speed-20) * ((target2 - (nMotorEncoder[motorFrontLeft] - posFL))/(target2+posFL) + 20)); //move at desired speed
//		}
//}

void moveDistanceRamp(int speed, int distance) {
	int target = inchesToEncoder(distance);
	resetEncoders();

	while(abs(nMotorEncoder[motorFrontLeft]) < abs(target)  //wait until position reached
		&& abs(nMotorEncoder[motorBackRight]) < abs(target)) {
			move(sgn(speed) * (((abs(speed) - 20) * (target-abs(nMotorEncoder[motorFrontLeft]))/target) + 20)); //move at desired speed
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
		error = target - nMotorEncoder[motorBackRight];
		speed = updatePID(turnPID, error, nMotorEncoder[motorBackRight]);
		writeDebugStreamLine("%f", speed);
		if(abs(speed) > 100)
			speed = sgn(speed) * 100;
		turn(-speed);
	}
	move(0);
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

task translateWithHeading() {
	const float threshold = 1;
	const float hugeThreshold = 20;
	float angleRad = (translateAngle + 45) * PI/180;
	float xaccel = 0;
	float yaccel = 0;
	float xaccellast = 0;
	float yaccellast = 0;
	float xforcefactor = 0;
	float yforcefactor = 0;
	//float initialHeading = getHeading();
	while(translating) {
		//xaccel = getAccelX();
		//yaccel = getAccelY();
		//if(abs(xaccel - xaccellast) > 1){
		//	xforcefactor = (xaccel -xaccellast) *10;
		//}
		//else{
		//	xforcefactor=0;
		//	xaccellast = xaccel;
		//}
		//if(abs(yaccel - yaccellast) > 1){
		//	yforcefactor = (yaccel -yaccellast) *10;
		//}
		//else{
		//	yforcefactor=0;
		//	yaccellast = yaccel;
		//}
		//xforcefactor=100;
		//yforcefactor=0;
		float rot = 0;
		float heading = getHeading();
		writeDebugStreamLine("%f", heading);
		float error = heading - initialHeading;

		if(heading > threshold + initialHeading)
			rot = error + 10;
		else if(heading < threshold - initialHeading)
			rot = error - 10;

		if(abs(error) > hugeThreshold)
			hugeBump = true;
		//float pushAngle = 0;
		//if(xforcefactor == 0) {
		//	pushAngle = (sgn(yforcefactor) == 1 ? PI : 0);
		//}
		//else if(yforcefactor == 0) {
		//	pushAngle = -PI/2 * sgn(xforcefactor);
		//}
		//else {
		//	pushAngle = atan2(yforcefactor, xforcefactor);
		//}

		float pow1 = translateSpeed * sin(angleRad) + rot;
		float pow2 = translateSpeed * cos(angleRad) - rot;
		float pow3 = translateSpeed * cos(angleRad) + rot;
		float pow4 = translateSpeed * sin(angleRad) - rot;
		motor[motorFrontLeft] = pow1;
		motor[motorBackRight] = pow4;
		motor[motorFrontRight] = pow2;
		motor[motorBackLeft] = pow3;
		wait1Msec(5);
	}
	move(0);
	//correctToInitialHeading();
}

void translateRTHeading(int speed, int angle) {
	translateSpeed = speed;
	translateAngle = angle;
	translating = true;
	StartTask(translateWithHeading);
}

//void turnWithGyro(int speed, float target) {
//	const float threshold = 0.5;
//	speed = 130;
//	while(heading < target - threshold || heading > target + threshold) {
//		float heading = getHeading();
//		speed =
//		writeDebugStreamLine("heading: %f, speed: %d", heading, speed);
//		turn(speed);
//		wait1Msec(5);
//	}
//	turn(0);
//	PlaySound(soundBeepBeep);
//	while(true);
//}

void turnWithGyro(int speed, int angle) {
	float target = initialHeading - angle;
	if(target > 180)
		target-=360;
	if(target < -180)
		target+=360;

	const int threshold = 1;
	while(abs(getHeading() - target) > threshold) {
		writeDebugStreamLine("%f", getHeading());
		if(getHeading() > target + threshold)
			turn((getHeading()-target)/360.0 * 100.00 + 35);
		if(getHeading() < target - threshold)
			turn((getHeading()-target)/360.0 * 100.00 - 35);
	}
	turn(0);
	initialHeading = target;
}

//void turnToHeading(int speed, float heading) {
//	const float threshold = 0.5;
//	while(heading < target - threshold || heading > target + threshold) {
//		float heading = getHeading();
//		writeDebugStreamLine("heading: %f, speed: %d", heading, speed);
//		turn(speed );
//		wait1Msec(5);
//	}
//	turn(0);
//}

void translateXY(int fwd, int right) {//It's a lot easier to use RT.
	motor[motorFrontLeft] = fwd + right;
	motor[motorBackRight] = fwd + right;
	motor[motorFrontRight] = fwd - right;
	motor[motorBackLeft] = fwd - right;
}

void translateXYAccel(int fwd, int right) {
	bool translating = true;
	float xaccel = 0;
	float yaccel = 0;
	float xaccellast = 0;
	float yaccellast = 0;
	float xforcefactor = 0;
	float yforcefactor = 0;

	while(translating) {
		xaccel = getAccelX();
		yaccel = getAccelY();
		if(abs(xaccel - xaccellast) >= 1){
			xforcefactor = (xaccel -xaccellast) *50;
		}
		else{
			xforcefactor=0;
			xaccellast = xaccel;
		}
		if(abs(yaccel - yaccellast) >= 1){
			yforcefactor = (yaccel -yaccellast) *50;
		}
		else{
			yforcefactor=0;
			yaccellast = yaccel;
		}

		motor[motorFrontLeft] = (fwd - yforcefactor) + (right - xforcefactor);
		motor[motorBackRight] = (fwd - yforcefactor) + (right - xforcefactor);
		motor[motorFrontRight] = (fwd - yforcefactor) - (right - xforcefactor);
		motor[motorBackLeft] = (fwd - yforcefactor) - (right - xforcefactor);
		wait10Msec(10);
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

void translateDistanceHeading(int speed, int angle, int distance) {
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
			translateRTHeading(speed, angle); //move at desired speed
		}

	translating = false;
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
