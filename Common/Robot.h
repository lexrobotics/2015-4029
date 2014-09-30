typedef struct {
	float x, y, o;
	float target_x, target_y, target_o;
	int speed_left;
	int speed_right;
	int pathIndex;
	int path[32];
} RobotState;

struct {
	RobotState state;
	RobotState prevState;
} Robot;
