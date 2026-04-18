/*---------------------------------------------------------------------------
   DispEst.h - Disparity Estimation Class Header
  ---------------------------------------------------------------------------
   Author: Charles Leech
   Email: cl19g10 [at] ecs.soton.ac.uk
   Copyright (c) 2016 Charlie Leech, University of Southampton.
  ---------------------------------------------------------------------------*/
#include "ComFunc.h"
#include "CVC.h"
#include "CVF.h"
#include "DispSel.h"
#include "PP.h"
#include "fastguidedfilter.h"
//
// Top-level Disparity Estimation Class
//
class DispEst
{
public:
    DispEst(cv::Mat l, cv::Mat r, const int d, int t, bool ocl);
    ~DispEst(void);

    //DispSel
    cv::Mat lDisMap;
    cv::Mat rDisMap;

    //Public Methods
	int setInputImages(cv::Mat l, cv::Mat r);
	int setThreads(unsigned int newThreads);
	void setSubsampleRate(unsigned int newRate) {subsample_rate = newRate;};
	int printCV(void);

    int CostConst();
    int CostConst_CPU();

    int CostFilter();
    int CostFilter_CPU();
    int CostFilter_FGF();


    int DispSelect_CPU();

    int PostProcess_CPU();

private:
    //Private Variable
    cv::Mat lImg;
    cv::Mat rImg;

    int hei;
    int wid;
    int maxDis;
    int threads;
    unsigned int subsample_rate = 4;

	//CVC
    cv::Mat lGrdX;
    cv::Mat rGrdX;
	//CVC & CVF
//    Mat lcostVol_cvc;
//    Mat rcostVol_cvc;
    cv::Mat* lcostVol;
    cv::Mat* rcostVol;
    //CVF
//    Mat* lImg_rgb;
//    Mat* rImg_rgb;
//    Mat* mean_lImg;
//    Mat* mean_rImg;
//    Mat* var_lImg;
//    Mat* var_rImg;
    //PP
    cv::Mat lValid;
    cv::Mat rValid;

    CVC* constructor;
    CVF* filter;
    DispSel* selector;
    PP* postProcessor;

    //Private Methods
    //None
};
