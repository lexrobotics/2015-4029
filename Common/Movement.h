
/*
Motor business
*/
void fullStop(){
	motor[motorFrontLeft] = 0;
	motor[motorFrontRight] = 0;
	motor[motorBackLeft] = 0;
	motor[motorBackRight] = 0;
}

void turnInPlace(int speed){
	motor[motorFrontLeft] = -speed;
	motor[motorBackRight] = speed;
	motor[motorFrontRight] = speed;
	motor[motorBackLeft] = -speed;
}

void translateRT(int speed, int angle) {
	if(speed > 60)
		speed = 60;
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

//Turns the ultrasonic indicated to the angle indicated.
//Asynchronous, so you have to wait a bit.
void turnUltra(int servo_index, int angle) {
	int scaled = angle * 256.0/180.0;
	int ZERO;
	switch(servo_index) {
		case 0:
			ZERO = 192;
			servo[servo1] = ZERO - scaled;
			break;
		case 1:
			ZERO = 150;
			servo[servo2] =  ZERO + scaled ;
			break;
	}
}


/*
Hybrid functions
*/


void tillSense(int speed, int angle, bool see_now, int threshold, tSensors sonar){
		while((SensorValue[sonar]<threshold)==see_now){
					translateRT(speed,angle);
		}

		translateRT(0, 0);
}

void changeDetection(int speed, int angle, int jumpThreshold, tSensors sonar) {
	int dist = SensorValue[sonar];

	while (abs(dist - SensorValue[sonar]) < jumpThreshold) {
		translateRT(speed, angle);
		dist = SensorValue[sonar];
	}

	translateRT(0, 0);
}

void parallel(int speed, int threshold, tSensors sensorA, tSensors sensorB){
	int valA = SensorValue[sensorA];
	int valB = SensorValue[sensorB];
	while (abs(valA - valB) > threshold) {
		valA = SensorValue[sensorA];
 		valB = SensorValue[sensorB];
		turnInPlace(speed * (valA>valB ? 1 : -1));
	}
	turnInPlace(0);
}


task main ()
{
turnInPlace(100);
}
