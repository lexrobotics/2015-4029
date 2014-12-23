#pragma config(Hubs,  S1, HTMotor,  HTMotor,  none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     motorFrontLeft, tmotorTetrix, PIDControl)
#pragma config(Motor,  mtr_S1_C1_2,     motorBackLeft, tmotorTetrix, PIDControl)
#pragma config(Motor,  mtr_S1_C2_1,     motorBackRight, tmotorTetrix, PIDControl, reversed)
#pragma config(Motor,  mtr_S1_C2_2,     motorFrontRight, tmotorTetrix, PIDControl, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "JoystickDriver.c"

task main()
{
	while(true) {
		getJoystickSettings(joystick);
		int x1 = joystick.joy1_x1;
		int y1 = joystick.joy1_y1;

		float angle = atan(x1/y1);
	}
}