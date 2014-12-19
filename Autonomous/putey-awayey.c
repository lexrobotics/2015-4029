#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  HTMotor)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     backUltra,      sensorSONAR)
#pragma config(Sensor, S3,     frontUltra,     sensorSONAR)
#pragma config(Sensor, S4,     HTIRS2,         sensorI2CCustom)
#pragma config(Motor,  mtr_S1_C1_1,     liftMotors,    tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     leftMotors,    tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     rightMotors,   tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     spinner,       tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_1,     liftStageOne,  tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C4_2,     liftStageTwo,  tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C3_1,    rearUltraServo,       tServoStandard)
#pragma config(Servo,  srvo_S1_C3_2,    bucketTilt,           tServoStandard)
#pragma config(Servo,  srvo_S1_C3_3,    bucketGate,           tServoStandard)
#pragma config(Servo,  srvo_S1_C3_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_6,    grabber,              tServoContinuousRotation)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

task main()
{
	bool stageOne = true;

	while(true) {
		nxtDisplayCenteredTextLine(2, "%d", nMotorEncoder[liftStageOne]);
		if(nNxtButtonPressed == 3) {
			stageOne = !stageOne;
			wait1Msec(500);
		}

		if(stageOne) {
			if(nNxtButtonPressed == 1)
				motor[liftStageOne] = 100;
			else if(nNxtButtonPressed == 2)
				motor[liftStageOne] = -100;
			else
				motor[liftStageOne] = 0;
		}
		else {
			if(nNxtButtonPressed == 1)
				motor[liftStageTwo] = -100;
			else if(nNxtButtonPressed == 2)
				motor[liftStageTwo] = 100;
			else
				motor[liftStageTwo] = 0;
		}
	}
}
