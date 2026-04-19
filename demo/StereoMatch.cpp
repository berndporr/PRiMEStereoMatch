/*---------------------------------------------------------------------------
   StereoMatch.cpp - Stereo Matching Application Class
  ---------------------------------------------------------------------------
   Author: Charles Leech
   Email: cl19g10 [at] ecs.soton.ac.uk
   Copyright (c) 2016 Charlie Leech, University of Southampton.
  ---------------------------------------------------------------------------*/

/*
 * Copyright (c) 2026 Bernd Porr, University of Glasgow
 */

#include "StereoMatch.h"

// #############################################################################
// # SM Preprocessing that we don't want to repeat
// #############################################################################
StereoMatch::StereoMatch(int argc, const char *argv[], int gotOpenCLDev)
{
#ifdef DEBUG_APP
	std::cout << "Stereo Matching for Depth Estimation." << std::endl;
	std::cout << "Preprocessing for Stereo Matching." << std::endl;
#endif // DEBUG_APP

	// #########################################################################
	// # Setup - check input arguments
	// #########################################################################
	if (parse_cli(argc, argv))
		exit(1);

	maxDis = 64;
	de_mode = OCL_DE;
	num_threads = MAX_CPU_THREADS;
	gotOCLDev = gotOpenCLDev;
	mask_mode = MASK_NONOCC;
	error_threshold = 4;
	scale_factor = 3;

	cvc_time_avg = 0;
	frame_count = 0;

	std::cout << "Loading " << curr_dataset << " as the default dataset." << std::endl;
	if (update_dataset(curr_dataset))
		exit(1);

	// #########################################################################
	// # End of Preprocessing (that we don't want to repeat)
	// #########################################################################
	printf("End of Preprocessing\n");
	printf("StereoMatch Application Initialised\n");
	return;
}

StereoMatch::~StereoMatch(void)
{
	printf("Application Shut down\n");
}

// #############################################################################
// # Complete GIF stereo matching process
// #############################################################################
int StereoMatch::compute(float &de_time_ms)
{
#ifdef DEBUG_APP
	std::cout << "Computing Depth Map" << std::endl;
#endif // DEBUG_APP

	float start_time = get_rt();
	input_data_m.lock();

	// #########################################################################
	// # Start of Disparity Map Creation
	// #########################################################################

	SMDE->setInputImages(lFrame, rFrame);
	SMDE->setThreads(num_threads);
	SMDE->setSubsampleRate(subsample_rate);
	SMDE->process();

	// #########################################################################
	// # End of Disparity Map Creation
	// #########################################################################

	// ******** Display Disparity Maps  ******** //
	SMDE->lDisMap.convertTo(lDispMap, CV_8U, scale_factor); // scale factor used to compare error with ground truth
	SMDE->rDisMap.convertTo(rDispMap, CV_8U, scale_factor);

	cv::cvtColor(lDispMap, leftDispMap, cv::COLOR_GRAY2RGB);
	cv::cvtColor(lDispMap, rightDispMap, cv::COLOR_GRAY2RGB);
	// ******** Display Disparity Maps  ******** //

	frame_count++;
#ifdef DEBUG_APP_MONITORS
	de_time_ms = (get_rt() - start_time) / 1000;
	printf("DE Time:\t %4.2f ms\n", de_time_ms);
#endif // DEBUG_APP_MONITORS

#ifdef DEBUG_APP
	cv::imwrite("leftDisparityMap.png", leftDispMap);
	cv::imwrite("rightDisparityMap.png", rightDispMap);
#endif

	if (ground_truth_data)
	{
		// Check pixel errors against ground truth depth map here.
		// Can only be done with images as golden reference is required.
		cv::absdiff(lDispMap, gtFrame, eDispMap);
		eDispMap(cv::Rect(0, 0, maxDis + 1, eDispMap.rows)).setTo(cv::Scalar(0));
		cv::threshold(eDispMap, eDispMap, error_threshold * (CHAR_MAX / maxDis), 255, cv::THRESH_TOZERO);

		if (mask_mode == MASK_DISC)
		{
			errMask = cv::imread(mask_disc_filename, cv::IMREAD_GRAYSCALE);
			cv::threshold(errMask, errMask, 254, 255, cv::THRESH_TOZERO); // set any grey to black
			eDispMap = eDispMap.mul(errMask, 1 / 255.f);
		}
		else if (mask_mode == MASK_NONOCC)
		{
			errMask = cv::imread(mask_occl_filename, cv::IMREAD_GRAYSCALE);
			eDispMap = eDispMap.mul(errMask, 1 / 255.f);
		}
		cvtColor(eDispMap, errDispMap, cv::COLOR_GRAY2RGB);

		float avg_err = cv::mean(eDispMap)[0] / (CHAR_MAX / maxDis);
		unsigned int num_bad_pixels = (unsigned int)cv::countNonZero(eDispMap);
		float num_pixels = gtFrame.cols * gtFrame.rows;
#ifdef DEBUG_APP_MONITORS
		printf("%%BP = %.2f%% \t Avg Err = %.2f\n", num_bad_pixels * 100 / num_pixels, avg_err);
#endif // DEBUG_APP_MONITORS
	}

	input_data_m.unlock();

	imshow("InputOutput", display_container);
	return 0;
}

int StereoMatch::update_dataset(std::string dataset_name)
{
	curr_dataset = dataset_name;
	string data_dir = "data/";
	if ((!dataset_name.compare("Cones")) || (!dataset_name.compare("Teddy")))
	{
		left_img_filename = data_dir + dataset_name + std::string("/im2.png");
		right_img_filename = data_dir + dataset_name + std::string("/im6.png");
		gt_img_filename = data_dir + dataset_name + std::string("/disp2.png");
		mask_occl_filename = data_dir + dataset_name + std::string("/occl.png");
		mask_disc_filename = data_dir + dataset_name + std::string("/occ_and_discont.png");
		mask_mode_next = MASK_NONOCC;
		scale_factor_next = 4;
	}
	else if (!user_dataset)
	{
		left_img_filename = data_dir + dataset_name + std::string("/view1.png");
		right_img_filename = data_dir + dataset_name + std::string("/view5.png");
		gt_img_filename = data_dir + dataset_name + std::string("/disp1.png");
		mask_mode_next = NO_MASKS;
		scale_factor_next = 3;
	}
	else
	{
		// Filenames for a user dataset are specified in the CLI
		scale_factor_next = 4;
	}

	input_data_m.lock();
	lFrame = cv::imread(left_img_filename, cv::IMREAD_COLOR);
	if (lFrame.empty())
	{
		std::cout << "Failed to read left image \"" << left_img_filename << "\"" << std::endl;
		std::cout << "Exiting" << std::endl;
		return -1;
	}
	else
	{
		std::cout << "Loaded Left: " << left_img_filename << std::endl;
	}
	rFrame = cv::imread(right_img_filename, cv::IMREAD_COLOR);
	if (rFrame.empty())
	{
		std::cout << "Failed to read right image \"" << right_img_filename << "\"" << std::endl;
		std::cout << "Exiting" << std::endl;
		return -1;
	}
	else
	{
		std::cout << "Loaded Right: " << right_img_filename << std::endl;
	}

	if (ground_truth_data)
	{
		gtFrame = cv::imread(gt_img_filename, cv::IMREAD_GRAYSCALE);
		if (gtFrame.empty())
		{
			std::cout << "Failed to read ground truth image \"" << gt_img_filename << "\"" << std::endl;
			std::cout << "Exiting" << std::endl;
			return -1;
		}
		else
		{
			std::cout << "Loaded Ground Truth: " << gt_img_filename << std::endl;
		}
	}
	else
	{
		gtFrame = cv::Mat(lFrame.rows, lFrame.cols, CV_8UC1);
	}

	eDispMap = cv::Mat(lFrame.rows, lFrame.cols, CV_8UC1);
	mask_mode = mask_mode_next;
	if (mask_mode != NO_MASKS)
		errMask = cv::imread(mask_occl_filename, cv::IMREAD_GRAYSCALE);

	imgDisparity16S = cv::Mat(lFrame.rows, lFrame.cols, CV_16S);
	blankDispMap = cv::Mat(rFrame.rows, rFrame.cols, CV_8UC3, cv::Scalar(0, 0, 0));
	update_display();
	SMDE = std::make_shared<PrimeStereoMatch>(lFrame.rows, lFrame.cols, maxDis, num_threads);
	error_threshold = (error_threshold / scale_factor) * scale_factor_next;
	scale_factor = scale_factor_next;

	input_data_m.unlock();
	return 0;
}

int StereoMatch::update_display(void)
{
	if (ground_truth_data)
	{
		display_container = cv::Mat(lFrame.rows * 2, lFrame.cols * 3, CV_8UC3, cv::Scalar(0, 0, 0));
		gtDispMap = cv::Mat(display_container, cv::Rect(lFrame.cols * 2, 0, lFrame.cols, lFrame.rows));			   // Top Far Right
		errDispMap = cv::Mat(display_container, cv::Rect(lFrame.cols * 2, lFrame.rows, lFrame.cols, lFrame.rows)); // Bottom Far Right
	}
	else
	{
		display_container = cv::Mat(lFrame.rows * 2, lFrame.cols * 2, CV_8UC3, cv::Scalar(0, 0, 0));
	}

	leftInputImg = cv::Mat(display_container, cv::Rect(0, 0, lFrame.cols, lFrame.rows));					 // Top Left
	rightInputImg = cv::Mat(display_container, cv::Rect(lFrame.cols, 0, lFrame.cols, lFrame.rows));			 // Top Right
	leftDispMap = cv::Mat(display_container, cv::Rect(0, lFrame.rows, lFrame.cols, lFrame.rows));			 // Bottom Left
	rightDispMap = cv::Mat(display_container, cv::Rect(lFrame.cols, lFrame.rows, lFrame.cols, lFrame.rows)); // Bottom Right

	lFrame.copyTo(leftInputImg);
	rFrame.copyTo(rightInputImg);
	cv::cvtColor(gtFrame, gtDispMap, cv::COLOR_GRAY2RGB);
	imshow("InputOutput", display_container);
	return 0;
}

int StereoMatch::parse_cli(int argc, const char *argv[])
{
	if (argc == 1)
	{
		fprintf(stderr, "Usage: %s [[left.png right.png] disparity.png]\n", argv[0]);
		curr_dataset = dataset_names[2];
		ground_truth_data = true;
		std::cout << "Demo mode. Daset used: " << curr_dataset << std::endl;
		return 0;
	}
	if (argc == 2)
	{
		std::cerr << "Can't just provide one image. We need a stereo pair." << std::endl;
		return 1;
	}
	if (argc > 2)
	{
		std::cout << "Stereo images provided." << std::endl;
		left_img_filename = argv[1];
		right_img_filename = argv[2];
		curr_dataset = "User";
		user_dataset = true;
	}
	if (argc > 3)
	{
		gt_img_filename = argv[3];
		ground_truth_data = true;
	}
	return 0;
}
