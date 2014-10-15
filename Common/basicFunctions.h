int getXAtIndex (Robot* r,int index){
	return r -> path[index * 2];
}
int getYAtIndex (Robot* r,int index){
	return r -> path[index * 2 + 1];
}

float getAngle(int aX, int aY, int bX, int bY){
	float xDiff = bX - aX;
	float yDiff = bY - aY;
	return atan2(yDiff,xDiff) * (180/PI);
}

float getDistance(int aX, int aY, int bX, int bY){
	return sqrt(pow((aX - bX), 2.0) + pow((aY - bY), 2.0));
}

void changeHeading(float current, float desired){
}

void drive(float speed, float distance){
}
