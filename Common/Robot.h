#ifndef ROBOT_H
#define ROBOT_H

typedef struct {
	float x, y, o;
	float target_x, target_y, target_o;
	int speed_left;
	int speed_right;
	int pathIndex;
	int path[32][2];
} RobotState;

struct {
	RobotState state;
	RobotState prevState;
} Robot;

void copyRobotState(RobotState *newState, RobotState oldState) {
	newState->x = oldState.x;
	newState->y = oldState.y;
	newState->o = oldState.o;

	newState->target_x = oldState.target_x;
	newState->target_y = oldState.target_y;
	newState->target_o = oldState.target_o;

	newState->speed_left = oldState.speed_left;
	newState->speed_right = oldState.speed_right;

	newState->pathIndex = oldState.pathIndex;
	newState->path = oldState.path;
}

#endif
