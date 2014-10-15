#include "../Common/Robot.h"

task main(){
	Robot robot;
	getGoal()
}

/*void setDestination(Robot* robot,int next_index){
	int current_x = getXAtIndex(robot -> state, next_index-1);
	int current_y = getYAtIndex(robot -> state, next_index-1);
	int next_x = getXAtIndex(robot -> state, next_index);
	int next_y = getYAtIndex(robot -> state, next_index);

	robot->state->desired_heading = getAngle(current_x,current_y,next_x,next_y);
	robot->state->next_location[0] = next_x;
	robot->state->next_location[1] = next_y;
}*/
