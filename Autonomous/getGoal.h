#include "../Common/basicFunctions.h"

void getGoal(Robot* robot){
	swerveToGoal(robot);
	grabGoal();
	returnGoalToRamp();
}
void swerveToGoal(Robot* robot){
	int goalX, goalY;
	moveTo(goalX,goalY,-50,STRAIGHT);
	moveTo(goalX,goalY,0,SWERVE);
	grab();

	//turn to goals
	//drive 50cm short
	//start swervindetecting

}
void moveToAngle(float angle){
	int direction = (Robot->heading>angle) - ((Robot->heading>angle)-1)*-1;
	while(abs(Robot->heading-angle)>5){
		left = 100*direction*-1;
		right = 100*direction;
	}
	//#PID MAGIC NEEDED
}
void move(int left,int right){
	motor[left] = left;
	motor[right]=right;
}
void moveTo(int goalX, int goalY, int shoot, int mode){
	int distance = getDistance(robot -> x, robot -> y, goalX, goalY);
	int angle = getAngle(robot -> x, robot -> y, goalX, goalY);
	moveToAngle(angle);
	//Then a switch for the modes and a different move function for each mode
	//done
}
