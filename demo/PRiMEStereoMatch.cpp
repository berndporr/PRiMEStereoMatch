/*---------------------------------------------------------------------------
   Author: Charles Leech
   Email: cl19g10 [at] ecs.soton.ac.uk
   Copyright (c) 2016 Charlie Leech, University of Southampton.
  ---------------------------------------------------------------------------
   Copyright (c) 2026 Bernd Porr, University of Glasgow
   Email: bernd.porr@glasgow.ac.uk
  */
#include "StereoMatch.h"
#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>

int main(int argc, const char *argv[])
{
	StereoMatch sm(argc, argv);
	sm.compute();
    sm.display();
	return 0;
}
