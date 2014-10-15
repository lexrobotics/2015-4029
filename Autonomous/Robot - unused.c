typedef struct Robot {
	float* location;
	float* next_location;
	float heading;
	float desired_heading;
	int* speed;
	int pathIndex;
	int* path;
} Robot;
