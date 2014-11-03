#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  HTMotor)
#pragma config(Sensor, S2,     backUltra,      sensorSONAR)
#pragma config(Sensor, S3,     frontUltra,     sensorSONAR)
#pragma config(Motor,  mtr_S1_C1_1,     liftMotors,    tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     leftMotors,    tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     rightMotors,   tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_2,     scoop,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_1,     tilt,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_2,     nomotor,       tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C3_1,    grabber,              tServoStandard)
#pragma config(Servo,  srvo_S1_C3_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_6,    servo6,               tServoNone)

#define AUTO_COMPETITION

#include "JoystickDriver.c"
#include "Autonomous.c"
#include "paths/Ramp.c"

/* Autonomous.c
   Final layer of abstraction
*/

void createTeleopConfigFile(string &sExecutableName);

task main() {
	nNxtExitClicks = 3;

	string teleopFileName = "TeleOp.c";
	const int NUM_PATHS = 2;
	const string PATH_NAMES[NUM_PATHS] = {"Ramp", "Floor"};
	const int MENU_ENTRIES = 2;
	string path = PATH_NAMES[0];
	int delay = 0;

	int step = 0;
	int selections[MENU_ENTRIES];
	while(step < MENU_ENTRIES+1) {
		if(nNxtButtonPressed == 1) selections[step]++;
		if(nNxtButtonPressed == 2) selections[step]--;
		if(nNxtButtonPressed == 0) step--;
		if(nNxtButtonPressed == 3) step++;
		if(nNxtButtonPressed!= -1) wait1Msec(200); // give time to lift off finger

		if(step == -1)
			return;
		else if(step == 0) {
			if(selections[step] >= NUM_PATHS)
				selections[step] = 0;
			else if(selections[step] < 0)
				selections[step] = NUM_PATHS - 1;
			nxtDisplayCenteredTextLine(2, "%s", PATH_NAMES[selections[step]]);
		}
		else if(step == 1) {
			if(selections[step] < 0)
				selections[step] = 0;
			nxtDisplayCenteredTextLine(2, "%d", selections[step]);
		}
	}
	//createTeleopConfigFile(teleopFileName);
	waitForStart();
	Ramp();
}

// snagged from ProgramChooser.c
void createTeleopConfigFile(string &sExecutableName)
{
	char sTextFileName[] = "FTCConfig.txt"; // Name of the file containg tele-op name

	TFileIOResult nIoResult;
	TFileHandle hFileHandle;
	short nFileSize;

	// Erase existing file
  do // Make a loop in case, due to error, there are multiple copies
  {
  	Delete(sTextFileName, nIoResult);
  } while (nIoResult == ioRsltSuccess);

  // Create the file
  nFileSize = strlen(sExecutableName) + 4;
  OpenWrite(hFileHandle, nIoResult, sTextFileName, nFileSize);
  WriteText(hFileHandle, nIoResult, sExecutableName);
  WriteText(hFileHandle, nIoResult, ".rxe");
  Close(hFileHandle, nIoResult);

  // Display Message
  string sMessage = (nIoResult == ioRsltSuccess) ? "File Created" : "File Error";
  //displayCommandProgress(sMessage);
  nxtDisplayCenteredTextLine(2, "%s", sMessage);
  return;
}
