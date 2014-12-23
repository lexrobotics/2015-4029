
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



/*
Ultrasonic business
*/
//Gets ultrasonic with index (whatever)
int readUltra(int sensor_index){
	switch(sensor_index){
		case 0 :
			return SensorValue[frontUltra];
		case 1 :
			return SensorValue[backUltra];
		case 2 :
			break;
		case 3 :
			break;
	}
	return -1;
}

//Turns the ultrasonic indicated to the angle indicated.
//Asynchronous, so you have to wait a bit.
void turnUltra(int servo_index, int angle) {
	int scaled = angle * 256.0/180.0;
	int ZERO;
	switch(servo_index) {
		case 0:
			ZERO = 192;
			servo[frontUltraServo] = ZERO - scaled;
			break;
		case 1:
			ZERO = 150;
			servo[rearUltraServo] =  ZERO + scaled ;
			break;
	}
}


/*
Hybrid functions
*/


void tillSense(int speed, int angle, bool see_now, int threshold, int sensor_index){
		while((Ultrasonic(sensor_index)<threshold)==see_now){
					translateRT(speed,angle);
		}

		translateRT(0, 0);
}

void changeDetection(int speed, int angle, int jumpThreshold, int sensor_index) {
	int dist = Ultrasonic(sensor_index);

	while (abs(dist - Ultrasonic(sensor_index)) < jumpThreshold) {
		translateRT(speed, angle);
	}

	translateRT(0, 0);
}

void parallel(int speed, int threshold, int sensorA, int sensorB){
	int valA = Ultrasonic(sensorA);
	int valB = Ultrasonic(sensorB);
	while (abs(valA - valB) > threshold) {
		valA = Ultrasonic(sensorA);
 		valB = Ultrasonic(sensorB);
		turnInPlace(speed * (valA>valB ? 1 : -1));
	}
	turnInPlace(0);
}


task main ()
{
turnInPlace(100);
}
