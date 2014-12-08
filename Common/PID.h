typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float integral;
	float maxIntegral;
	float prevPosition;
} PID;

float updatePID(PID &pid, float error, float position) {
	float pTerm = pid.Kp * error;
	pid.integral += error;

	float iTerm = pid.Ki * pid.integral;
	float dTerm = pid.Kd * (position - pid.prevPosition);
	pid.prevPosition = position;

	return pTerm + iTerm - dTerm;
}
