/*---------------------------------------------------------------------------
   main.cpp - central Disparity Estimation Code
  ---------------------------------------------------------------------------
   Author: Charles Leech
   Email: cl19g10 [at] ecs.soton.ac.uk
   Copyright (c) 2016 Charlie Leech, University of Southampton.
  ---------------------------------------------------------------------------
   Author: Bernd Porr
   Email: bernd.porr@glasgow.ac.uk
  */
#include "StereoMatch.h"
#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>

int main(int argc, const char *argv[])
{
	printf("Starting Stereo Matching Application.\n");

	StereoMatch sm(argc, argv);
	sm.compute();

	imshow("PrimeStereoMatch", sm.display_container);
	fprintf(stderr,"Press any key.\n");
	waitKey(0);

	printf("MAIN: Disparity Estimation Halted\n");
	return 0;
}
