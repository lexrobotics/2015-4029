 #ifndef TOUCH_H
#define TOUCH_H

#include "drivers/hitechnic-superpro.h"

bool armSwitch, sideSwitch;
int dummy;

void readAllSwitches() {
	armSwitch = (HTSPBreadADC(HTSPB, 2, 10) == 1023);
	sideSwitch = (HTSPBreadADC(HTSPB, 1, 10) == 1023);
}

#endif
