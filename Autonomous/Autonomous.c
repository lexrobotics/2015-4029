#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  none)
#pragma config(Hubs,  S2, HTServo,  HTServo,  HTMotor,  none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ,               sensorI2CMuxController)
#pragma config(Sensor, S3,     HTSMUX,         sensorI2CCustom)
#pragma config(Sensor, S4,     HTSPB,          sensorI2CCustomFast9V)
#pragma config(Motor,  mtr_S1_C1_1,     motorBackRight, tmotorTetrix, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     motorFrontRight, tmotorTetrix, PIDControl, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     motorBackLeft, tmotorTetrix, PIDControl, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     motorFrontLeft, tmotorTetrix, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C3_1,     motorH,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     harvester,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C3_1,     motorj,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C3_2,     conveyor,      tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S2_C1_1,    lift1,                tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C1_2,    lift2,                tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C1_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_6,    servo6,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_1,    frontTurret,          tServoStandard)
#pragma config(Servo,  srvo_S2_C2_2,    rearTurret,           tServoStandard)
#pragma config(Servo,  srvo_S2_C2_3,    kickstand,            tServoStandard)
#pragma config(Servo,  srvo_S2_C2_4,    grabber,              tServoContinuousRotation)
#pragma config(Servo,  srvo_S2_C2_5,    servo5,               tServoStandard)
#pragma config(Servo,  srvo_S2_C2_6,    servo6,               tServoStandard)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#define AUTO_COMPETITION

#include "JoystickDriver.c"
#include "paths/RampKickstand.c"
#include "paths/Kickstand.c"
#include "paths/SpeedyRamp.c"

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

bool AUTO_STARTED = false;

task emergencyAutoWatch() {
	waitForStart();
	AUTO_STARTED = true;
}

task main() {
	nNxtExitClicks = 3;
	StartTask(emergencyAutoWatch);
	string teleopFileName = "TeleOp.c";
	Options PATHS;
	PATHS.strings[0] = "Ramp";
	PATHS.strings[1] = "KickRamp";
	PATHS.strings[2] = "Kickstand";
	PATHS.len = 3;

	const int MENU_ENTRIES = 3;
	int delay = 0;
	int path = 0;
	int step = 0;
	int j=0;
	while(step < MENU_ENTRIES && !AUTO_STARTED) {
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
	nxtDisplayCenteredTextLine(2, "*****%s %ds******", path, delay);
	//createTeleopConfigFile(teleopFileName);
	waitForStart();
	pause(delay);
	if(path == 0)
		SpeedyRamp();
	else if(path == 1)
		RampKickstand();
	else
		Kickstand();
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
