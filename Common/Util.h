#ifndef UTIL_H
#define UTIL_H

void pause(float seconds) {
	wait1Msec(seconds * 1000);
}

int min(int a, int b)
{
	if (a < b)
		return a;
	else
		return b;
}

#endif /* UTIL_H */
