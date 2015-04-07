 #ifndef TOUCH_H
#define TOUCH_H

#include "drivers/hitechnic-superpro.h"

bool armSwitch, sideSwitch, tipSwitch;

void readAllSwitches() {
	armSwitch = (HTSPBreadADC(HTSPB, 2, 10) == 1023);
	sideSwitch = (HTSPBreadADC(HTSPB, 3, 10) == 1023);
	tipSwitch = (HTSPBreadADC(HTSPB, 1, 10) == 1023);
}

#endif
