#include "drivers/hitechnic-gyro.h"
#include "common/Robot.h"

int _localisation_left_encoder;
int _localisation_right_encoder;
int _localisation_last_update;
float _localisation_o = 0;

task gyroTask() {
	int curRate = 0;
	float delTime = 0;
	while (true) {
    time1[T1] = 0;
    curRate = HTGYROreadRot(gyro);
    if (abs(curRate) > 3) {
      _localisation_o += curRate * delTime; //Approximates the next heading by adding the rate*time.
      if (_localisation_o > 360) _localisation_o -= 360;
      else if (_localisation_o < -360) _localisation_o += 360;
    }
    wait1Msec(5);
    delTime = ((float)time1[T1]) / 1000; //set delta (zero first time around)
  }
}

void localisation_init() {
	HTGYROstartCal(gyroTask);
  wait1Msec(1000);
  PlaySound(soundBeepBeep);
  StartTask(gyro);
	nMotorEncoder[right] = nMotorEncoder[left] = 0;
	_localisation_left_encoder = nMotorEncoder[left];
	_localisation_right_encoder = nMotorEncoder[right];
	_localisation_last_update = time1[T1];
}

void localisation_update(RobotState &state) {
	int dt = time1[T1] - _localisation_last_update;
	int curRate = HTGYROreadRot(gyro);
  if (abs(curRate) > 3) {
  	state.o += curRate * dt/1000.0; //Approximates the next heading by adding the rate*time.
     if (state.o > 360) state.o -= 360;
     else if (state.o < -360) state.o += 360;
  }
  int netEncoderChange = (nMotorEncoder[left] + nMotorEncoder[right])/2;
  int d = (netEncoderChange/1440.0) * 4.0 * PI;
 	state.x += cos(state.o) * d;
 	state.y += sin(state.o) * d;
	state.o = _localisation_o;
	nMotorEncoder[left] = 0;
	nMotorEncoder[right] = 0;
	_localisation_last_update = time1[T1];
}
