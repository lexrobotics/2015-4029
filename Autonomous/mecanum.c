void translate(int speed, int angle);

void translate(int speed, int angle)
	angle += 45;
	if(angle > 360)
		angle -= 360;

	int pair1 = cos(angle * PI / 180.0);
	int pair2 = sin(angle * PI/ 180.0);
	motor[leftFront] 	= pair1;
	motor[rightBack] 	= pair1;
	motor[leftBack]  	= pair2;
	motor[rightFront] = pair2;
}
