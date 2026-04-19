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
StereoMatch::StereoMatch(int argc, const char *argv[], int gotOpenCLDev) : end_de(false), user_dataset(false), ground_truth_data(false)
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

	unsigned int cam_height = 376; //  376  (was 480),  720, 1080, 1242
	unsigned int cam_width = 1344; // 1344 (was 1280), 2560, 3840, 4416

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
	// # SGBM Mode Setup
	// #########################################################################
	setupOpenCVSGBM(lFrame.channels(), maxDis);
	imgDisparity16S = cv::Mat(lFrame.rows, lFrame.cols, CV_16S);
	blankDispMap = cv::Mat(rFrame.rows, rFrame.cols, CV_8UC3);

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
	if (MatchingAlgorithm == STEREO_SGBM)
	{
#ifdef DEBUG_APP
		printf("MatchingAlgorithm == STEREO_SGBM\n");
#endif // DEBUG_APP
		if ((lFrame.type() & CV_MAT_DEPTH_MASK) != CV_8U)
		{
			lFrame.convertTo(lFrame, CV_8U, 255);
			rFrame.convertTo(rFrame, CV_8U, 255);
		}

		// Compute the disparity map
		ssgbm->compute(lFrame, rFrame, imgDisparity16S);
		minMaxLoc(imgDisparity16S, &minVal, &maxVal); // Check its extreme values

		// Load the disparity map to the display
		imgDisparity16S.convertTo(lDispMap, CV_8U, 255 / (maxVal - minVal));
		lDispMap = (lDispMap / 4) * scale_factor;
		cvtColor(lDispMap, leftDispMap, cv::COLOR_GRAY2RGB);
	}
	else if (MatchingAlgorithm == STEREO_GIF)
	{
#ifdef DEBUG_APP
		printf("MatchingAlgorithm == STEREO_GIF\n");
#endif // DEBUG_APP
		if ((lFrame.type() & CV_MAT_DEPTH_MASK) != CV_32F)
		{
			lFrame.convertTo(lFrame, CV_32F, 1 / 255.0f);
			rFrame.convertTo(rFrame, CV_32F, 1 / 255.0f);
		}
		SMDE->setInputImages(lFrame, rFrame);
		SMDE->setThreads(num_threads);
		SMDE->setSubsampleRate(subsample_rate);

		// ******** Disparity Estimation Code ******** //
#ifdef DEBUG_APP
		std::cout << "Disparity Estimation Started..." << std::endl;
#endif // DEBUG_APP

		cvc_time = get_rt();
		SMDE->CostConst();
		cvc_time = get_rt() - cvc_time;

		cvf_time = get_rt();
		SMDE->CostFilter_FGF();
		cvf_time = get_rt() - cvf_time;

		dispsel_time = get_rt();
		SMDE->DispSelect_CPU();
		dispsel_time = get_rt() - dispsel_time;

		pp_time = get_rt();
		SMDE->PostProcess_CPU();
		pp_time = get_rt() - pp_time;
#ifdef DEBUG_APP
		std::cout << "Disparity Estimation Complete." << std::endl;
#endif // DEBUG_APP

		// ******** Display Disparity Maps  ******** //
		SMDE->lDisMap.convertTo(lDispMap, CV_8U, scale_factor); // scale factor used to compare error with ground truth
		SMDE->rDisMap.convertTo(rDispMap, CV_8U, scale_factor);

		cv::cvtColor(lDispMap, leftDispMap, cv::COLOR_GRAY2RGB);
		cv::cvtColor(lDispMap, rightDispMap, cv::COLOR_GRAY2RGB);
		// ******** Display Disparity Maps  ******** //

#ifdef DEBUG_APP_MONITORS
		cvc_time_avg = (cvc_time_avg * frame_count + cvc_time) / (frame_count + 1);
		printf("STEREO GIF Module Times:\n");
		printf("CVC Time:\t %4.2f ms   Avg Time:\t %4.2f\n", cvc_time / 1000, cvc_time_avg / 1000);
		printf("CVF Time:\t %4.2f ms\n", cvf_time / 1000);
		printf("DispSel Time:\t %4.2f ms\n", dispsel_time / 1000);
		printf("PP Time:\t %4.2f ms\n", pp_time / 1000);
#endif // DEBUG_APP_MONITORS
		frame_count++;
	}
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
			if (MatchingAlgorithm == STEREO_SGBM)
				cv::cvtColor(errMask, rightDispMap, cv::COLOR_GRAY2RGB);
			eDispMap = eDispMap.mul(errMask, 1 / 255.f);
		}
		else if (mask_mode == MASK_NONOCC)
		{
			errMask = cv::imread(mask_occl_filename, cv::IMREAD_GRAYSCALE);
			if (MatchingAlgorithm == STEREO_SGBM)
				cv::cvtColor(errMask, rightDispMap, cv::COLOR_GRAY2RGB);
			eDispMap = eDispMap.mul(errMask, 1 / 255.f);
		}
		else
		{
			if (MatchingAlgorithm == STEREO_SGBM)
				blankDispMap.copyTo(rightDispMap);
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

#ifdef DISPLAY
	imshow("InputOutput", display_container);
#endif
	return 0;
}

// #############################################################################
// # Camera resolution control
// #############################################################################
int StereoMatch::setCameraResolution(unsigned int height, unsigned int width)
{
	if (cap.get(CAP_PROP_FRAME_HEIGHT) != 376)
	{
		cap.set(CAP_PROP_FRAME_HEIGHT, 376); //  376  (was 480),  720, 1080, 1242
		cap.set(CAP_PROP_FRAME_WIDTH, 1344); // 1344 (was 1280), 2560, 3840, 4416
		cap.set(CAP_PROP_FPS, 30);			 // 376: {15, 30, 60, 100}, 720: {15, 30, 60}, 1080: {15, 30}, 1242: {15},

		if (cap.get(CAP_PROP_FRAME_HEIGHT) != 376)
		{
			std::cout << "Could not set correct frame resolution:" << std::endl;
			std::cout << "\t Target Height: " << 376 << std::endl;
			std::cout << "\t CAP_PROP_FRAME_HEIGHT: " << cap.get(CAP_PROP_FRAME_HEIGHT) << std::endl;
			std::cout << "\t CAP_PROP_FRAME_WIDTH: " << cap.get(CAP_PROP_FRAME_WIDTH) << std::endl;

			return 0; // Comment this line to search for vaild resolutions
			std::vector<Resolution> valid_res = resolution_search();
			cap.set(CAP_PROP_FRAME_HEIGHT, valid_res[0].height);
			cap.set(CAP_PROP_FRAME_WIDTH, valid_res[0].width);
			std::cout << "Set frame resolution to: " << valid_res[0].height << " x " << valid_res[0].width << std::endl;
		}
		else
		{
			std::cout << "Camera Settings:\n"
					  << std::endl;
			std::cout << "\t CAP_PROP_FPS: " << cap.get(CAP_PROP_FPS) << std::endl;
			std::cout << "\t CAP_PROP_FRAME_HEIGHT: " << cap.get(CAP_PROP_FRAME_HEIGHT) << std::endl;
			std::cout << "\t CAP_PROP_FRAME_WIDTH: " << cap.get(CAP_PROP_FRAME_WIDTH) << std::endl;
		}
	}

	return 0;
}

std::vector<Resolution> StereoMatch::resolution_search(void)
{
	unsigned int test_hei = 200, max_hei = 2162;
	float aspect_ratios[] = {4.f / 3.f, 16.f / 9.f};
	unsigned int stereo_multiplier = 2;
	unsigned int ret_hei, curr_hei, ret_wid;
	std::vector<Resolution> valid_res;

	for (int ar_idx = 0; ar_idx < sizeof(aspect_ratios) / sizeof(float); ++ar_idx)
	{
		for (int test_hei = 200; test_hei < max_hei; test_hei += 2)
		{
			// std::cout << "Testing: " << test_hei << " x " << (unsigned int)(test_hei*aspect_ratios[ar_idx]*stereo_multiplier) << " AR = " << aspect_ratios[ar_idx];
			cap.set(CAP_PROP_FRAME_HEIGHT, test_hei);
			cap.set(CAP_PROP_FRAME_WIDTH, (unsigned int)(test_hei * aspect_ratios[ar_idx] * stereo_multiplier));

			ret_hei = cap.get(CAP_PROP_FRAME_HEIGHT);
			// std::cout << " ret_hei: " << ret_hei << std::endl;

			if (ret_hei != curr_hei)
			{
				ret_wid = cap.get(CAP_PROP_FRAME_WIDTH);
				valid_res.push_back({ret_hei, ret_wid});
				std::cout << "Found new resolution: " << ret_hei << " x " << ret_wid << std::endl;
			}
			curr_hei = ret_hei;
		}
	}
	std::cout << "Valid resolutions: " << std::endl;
	for (auto res : valid_res)
		std::cout << "\t" << res.height << " x " << res.width << std::endl;
	return valid_res;
}

int StereoMatch::update_dataset(std::string dataset_name)
{
	curr_dataset = dataset_name;
std:
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
#ifdef DISPLAY
	update_display();
#endif // DISPLAY
	SMDE = std::make_shared<DispEst>(lFrame, rFrame, maxDis, num_threads, gotOCLDev);

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

// #############################################################################################
// # Setup for OpenCV implementation of Stereo matching using Semi-Global Block Matching (SGBM)
// #############################################################################################
int StereoMatch::setupOpenCVSGBM(int channels, int ndisparities)
{
	int mindisparity = 0;
	int SADWindowSize = 5;

	// Call the constructor for StereoSGBM
	ssgbm = StereoSGBM::create(
		mindisparity,								   // minDisparity = 0,
		ndisparities,								   // numDisparities = 16,
		SADWindowSize,								   // blockSize = 3,
		8 * channels * SADWindowSize * SADWindowSize,  // P1 = 0,
		32 * channels * SADWindowSize * SADWindowSize, // P2 = 0,
		1,											   // disp12MaxDiff = 0,
		63,											   // preFilterCap = 0,
		10,											   // uniquenessRatio = 0,
		100,										   // speckleWindowSize = 0,
		32,											   // speckleRange = 0,
		StereoSGBM::MODE_HH							   // mode = StereoSGBM::MODE_SGBM
	);

	return 0;
}

int StereoMatch::parse_cli(int argc, const char *argv[])
{
	if (argc == 1)
	{
		fprintf(stderr,"Usage: %s GIF|SGBM [[left.png right.png] disparity.png]\n",argv[0]);
		return 1;
	}
	if (argc > 1)
	{
		std::string alg_mode = argv[1];
		if (alg_mode == "GIF")
		{
			MatchingAlgorithm = STEREO_GIF;
		}
		std::cout << "Matching Algorithm: STEREO_GIF" << std::endl;
		if (alg_mode == "SGBM")
		{
			MatchingAlgorithm = STEREO_SGBM;
			std::cout << "Matching Algorithm: STEREO_SGBM" << std::endl;
		}
	}
	if (argc == 2)
	{
		curr_dataset = dataset_names[2];
		ground_truth_data = true;
		std::cout << "Daset used: " << curr_dataset << std::endl;
		return 0;
	}
	if (argc == 3)
	{
		std::cerr << "Can't just provide one image. We need a stereo pair." << std::endl;
		return 1;
	}
	if (argc > 3)
	{
		std::cout << "Stereo images provided." << std::endl;
		left_img_filename = argv[2];
		right_img_filename = argv[3];
		curr_dataset = "User";
		user_dataset = true;
	}
	if (argc > 4)
	{
		gt_img_filename = argv[4];
		ground_truth_data = true;
	}
	return 0;
}
