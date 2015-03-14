#ifndef SIXTYCM_H
#define SIXTYCM_H

void SixtyCM() {

	moveDistancePID(-70); // move down the ramp

	pause(0.1);
	incrementalParallel(25, 2, rearUS, frontUS); //Parallel to the wall
	pause(0.2);
	tillSense(100, 270, 4, true, frontUS); //???
	incrementalParallel(25, 2, rearUS, frontUS); //Parallel again to be extra sure
	pause(0.2);

	moveDistancePID(-40); // move further to shift to the tube into the pentagon slot

	grabTube(); //Lower the tube grabber
	moveDistance(50, 18);
	pause(0.3);

	move(0);

	turnUltra(0, 0);
	pause(0.3);
	tillSense(200, 90, true, 65, frontUS);
	pause(0.1);
	incrementalParallel(25, 2, rearUS, frontUS);
	pause(0.2);
	turnUltra(0, 90);
	pause(0.3);
	tillSense(100, 0, false, 50, frontUS);
}

#endif
