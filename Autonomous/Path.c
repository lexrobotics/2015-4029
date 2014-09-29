#include "Robot.c"

int getXAtIndex (Robot* r,int index){
	return r->path[index*2];
}
int getYAtIndex (Robot* r,int index){
	return r->path[index*2 + 1];
}

float getAngle(int aX, int aY, int bX, int bY){
	float xDiff = bX - aX;
	float yDiff = bY - aY;
	return atan2(yDiff,xDiff) * (180/PI);
}

void setDestination(Robot* robot,int next_index){
	int current_x = getXAtIndex(robot, next_index-1);
	int current_y = getYAtIndex(robot, next_index-1);
	int next_x = getXAtIndex(robot, next_index);
	int next_y = getYAtIndex(robot, next_index);

	robot->desired_heading = getAngle(current_x,current_y,next_x,next_y);
	robot->next_location[0] = next_x;
	robot->next_location[1] = next_y;
}

task main(){
	Robot r;

}
