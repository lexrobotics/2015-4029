#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  HTMotor)
#pragma config(Sensor, S2,     backUltra,      sensorSONAR)
#pragma config(Sensor, S3,     frontUltra,     sensorSONAR)
#pragma config(Sensor, S4,     touch,          sensorTouch)
#pragma config(Motor,  mtr_S1_C1_1,     liftMotors,    tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     leftMotors,    tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     rightMotors,   tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     spinner,       tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_1,     liftStageOne,  tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C4_2,     liftStageTwo,  tmotorTetrix, openLoop, encoder)
#pragma config(Servo,  srvo_S1_C3_1,    rearUltraServo,       tServoStandard)
#pragma config(Servo,  srvo_S1_C3_2,    bucketTilt,           tServoStandard)
#pragma config(Servo,  srvo_S1_C3_3,    bucketGate,           tServoStandard)
#pragma config(Servo,  srvo_S1_C3_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_6,    grabber,              tServoContinuousRotation)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#define AUTO_COMPETITION

#include "JoystickDriver.c"
#include "paths/Ramp.c"

/* Autonomous.c
   Final layer of abstraction
*/
typedef struct {
	string strings[16];
	int len;
} Options;

void createTeleopConfigFile(string &sExecutableName);
void finalCheck();
int selectInt(const string label, int prev);
int selectString(Options s, const string label, int prev);

task main() {
	nNxtExitClicks = 3;

	string teleopFileName = "TeleOp.c";
	const int NUM_PATHS = 2;
	Options PATHS;
	PATHS.strings[0] = "Ramp";
	//PATHS.strings[1] = "Parking";
	PATHS.len = 1;

	const int MENU_ENTRIES = 3;
	int delay = 0;
	int path = 0;
	int step = 0;
	int j=0;
	while(step < MENU_ENTRIES) {
		if(nNxtButtonPressed == 0) step--;
		if(nNxtButtonPressed == 3) step++;
		if(nNxtButtonPressed!= -1) wait1Msec(200); // give time to lift off finger

		if(step == -1)
			return;
		else if(step == 0) {
			path = selectString(PATHS, "Path", path);
		}
		else if(step == 1) {
			delay = selectInt("Delay", delay);
		}
		else {
			finalCheck();
		}
	}
	nxtDisplayCenteredTextLine(2, "Ready! %ds", delay);
	//createTeleopConfigFile(teleopFileName);
	waitForStart();
	pause(delay);
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

int selectInt(const string label, int prev) {
	int i = prev;
	nxtDisplayCenteredTextLine(2, "%s: %d", label, i);
	if(nNxtButtonPressed == 1) i++;
	if(nNxtButtonPressed == 2) i--;
	if(nNxtButtonPressed != -1) wait1Msec(200);
	if(i<0) i = 0;
	return i;
}

int selectString(Options s, const string label, int prev) {
	int i = prev;
	int len = s.len - 1;
	nxtDisplayCenteredTextLine(2, "%s: %s", label, s.strings[i]);
	if(nNxtButtonPressed == 1) i++;
	if(nNxtButtonPressed == 2) i--;
	if(nNxtButtonPressed != -1) wait1Msec(200);
	if(i<0) i = len;
	if(i>len) i = 0;
	return i;
}

void finalCheck() {
	return;
}
