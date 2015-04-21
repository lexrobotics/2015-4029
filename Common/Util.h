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

//int round(float f) {
//  if(f>0) return (int)(f + 0.5);
//	else    return (int)(f - 0.5);
//}

#endif /* UTIL_H */
