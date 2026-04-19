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

// Global variables
bool end_de = false;

void getDepthMap(StereoMatch &sm)
{
	int ret = 0;
	float de_time;

	while (!(end_de || ret))
	{
		ret = sm.compute(de_time);
	}
	return;
}

static void on_trackbar_err(int value, void *ptr)
{
	printf("HCI: Error threshold set to %d.\n", value);
}

void HCI(StereoMatch &sm)
{
	// User interface input handler
	char key = ' ';

	imshow("InputOutput", sm.display_container);

	cv::createTrackbar("Error Threshold", "InputOutput", &sm.error_threshold, 64, on_trackbar_err);
	on_trackbar_err(sm.error_threshold, (void *)4);

	while (key != 'q')
	{
		switch (key)
		{
		case 'h':
		{
			printf("|-------------------------------------------------------------------|\n");
			printf("| Input Options:\n");
			printf("| h: Display this help text.\n");
			printf("| q: Quit.\n");
			printf("|-------------------------------------------------------------------|\n");
			printf("| Control Options:\n");
			printf("|   1-8: Change thread/core number.\n");
			printf("|   a:   Switch matching algorithm: STEREO_GIF, STEREO_SGBM\n");
			printf("|   d:   Cycle between images datasets:\n");
			printf("|   d:   	Art, Books, Cones, Dolls, Laundry, Moebius, Teddy.n");
			printf("|   m:   Switch computation mode:\n");
			printf("|   m:      STEREO_GIF:  OpenCL <-> pthreads.\n");
			printf("|   m:      STEREO_SGBM: MODE_SGBM, MODE_HH, MODE_SGBM_3WAY\n");
			printf("|   -/=: Increase or decrease the error threshold\n");
			break;
		}
		case 'o':
		{
			if (sm.mask_mode == NO_MASKS)
			{
				printf("| o: Disparity error masks not provided for the chosen dataset.\n");
				break;
			}
			sm.mask_mode = (sm.mask_mode == MASK_NONE ? MASK_NONOCC : sm.mask_mode == MASK_NONOCC ? MASK_DISC
																									 : MASK_NONE);
			printf("| o: Disparity error mask set to: %s |\n", sm.mask_mode == MASK_NONE ? "None" : sm.mask_mode == MASK_NONOCC ? "Nonocc"
																																  : "Disc");
			break;
		}
		case 's':
		{
			sm.subsample_rate *= 2;
			if (sm.subsample_rate > 8)
				sm.subsample_rate = 2;
			printf("| =: Subsample rate changed to %d.\n", sm.subsample_rate);
			break;
		}
		}
		key = waitKey(0);
	}
	return;
}



int main(int argc, const char *argv[])
{
	namedWindow("InputOutput");
	printf("Starting Stereo Matching Application.\n");
	
	StereoMatch sm(argc, argv);

	std::thread de_thread;
	de_thread = std::thread([&](){getDepthMap(sm);});

	// User interface function
	HCI(sm);

	end_de = true;
	printf("MAIN: Quit signal sent\n");
	de_thread.join();

	printf("MAIN: Disparity Estimation Halted\n");
	return 0;
}


