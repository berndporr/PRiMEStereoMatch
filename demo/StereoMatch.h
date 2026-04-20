/*---------------------------------------------------------------------------
   StereoMatch.h - Stereo Matching Application Header
  ---------------------------------------------------------------------------
   Author: Charles Leech
   Email: cl19g10 [at] ecs.soton.ac.uk
   Copyright (c) 2016 Charlie Leech, University of Southampton.
  ---------------------------------------------------------------------------
   Copyright (c) 2026 Bernd Porr, University of Glasgow
   Email: bernd.porr@glasgow.ac.uk
*/
#ifndef STEREOMATCH_H
#define STEREOMATCH_H

#include "PrimeStereoMatch.h"

#define NO_MASKS 0
#define MASK_NONE 1
#define MASK_NONOCC 2
#define MASK_DISC 3

// #define DEBUG_APP
#define DEBUG_APP_MONITORS

static std::vector<std::string> dataset_names = std::vector<std::string>{"Art", "Books", "Cones", "Dolls", "Laundry", "Moebius", "Teddy"};

class StereoMatch
{
public:
	StereoMatch(int argc, const char *argv[]);
	~StereoMatch(void);

	void compute();
	void display();

private:
	int mask_mode = NO_MASKS;
	cv::Mat display_container;
	std::string left_img_filename, right_img_filename;
	std::string gt_img_filename, mask_occl_filename, mask_disc_filename;
	std::string curr_dataset;
	bool ground_truth_data = false;
	int scale_factor = 3;

	// Display Variables
	cv::Mat leftInputImg, rightInputImg;
	cv::Mat leftDispMap, rightDispMap;
	cv::Mat gtDispMap, errDispMap;
	cv::Mat blankDispMap;

	// local disparity map containers
	cv::Mat lDispMap, rDispMap, eDispMap;
	cv::Mat errMask;

	// input values
	int maxDis;

	// stage & process time measurements
	double cvc_time, cvf_time, dispsel_time, pp_time;
	double cvc_time_avg, cvf_time_avg, dispsel_time_avg, pp_time_avg;

	// Frame Holders & Camera object
	cv::Mat lFrame, rFrame, vFrame;

	// Image rectification maps
	cv::Mat mapl[2], mapr[2];
	cv::Rect cropBox;
	cv::Mat lFrame_rec, rFrame_rec;
	cv::Mat gtFrame;

	cv::Mat imgDisparity16S;

	std::shared_ptr<PrimeStereoMatch> SMDE;
	int num_threads = 1;

	int update_dataset(std::string dataset_name);
	bool user_dataset = false;

	// Stereo GIF Variables
	unsigned int subsample_rate = 4;

	int update_display(void);
	int parse_cli(int argc, const char *argv[]);
	float get_rt() const
	{
		struct timespec realtime;
		clock_gettime(CLOCK_MONOTONIC, &realtime);
		return (float)(realtime.tv_sec * 1000000 + realtime.tv_nsec / 1000);
	}
};

#endif // STEREOMATCH_H
