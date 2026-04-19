/*---------------------------------------------------------------------------
   DispEst.h - Disparity Estimation Class Header
  ---------------------------------------------------------------------------
   Author: Charles Leech
   Email: cl19g10 [at] ecs.soton.ac.uk
   Copyright (c) 2016 Charlie Leech, University of Southampton.
  ---------------------------------------------------------------------------*/
#include "prime_stereo_match/ComFunc.h"
#include "prime_stereo_match/CVC.h"
#include "prime_stereo_match/CVF.h"
#include "prime_stereo_match/DispSel.h"
#include "prime_stereo_match/PP.h"
#include "prime_stereo_match/fastguidedfilter.h"

//
// Top-level Disparity Estimation Class
//
class PrimeStereoMatch
{
public:
    PrimeStereoMatch(int height, int width, const int maxDisparity, int numThreads);
    ~PrimeStereoMatch(void);

    cv::Mat lDisMap;
    cv::Mat rDisMap;

    // Public Methods
    int setInputImages(cv::Mat l, cv::Mat r);
    int setThreads(unsigned int newThreads);
    void setSubsampleRate(unsigned int newRate) { subsample_rate = newRate; };
    void process();

private:
    int CostConst();
    int CostConst_CPU();
    int CostFilter_FGF();
    int DispSelect_CPU();
    int PostProcess_CPU();

    // Private Variable
    cv::Mat lImg;
    cv::Mat rImg;

    int hei;
    int wid;
    int maxDis;
    int threads;
    unsigned int subsample_rate = 4;

    // CVC
    cv::Mat lGrdX;
    cv::Mat rGrdX;
    cv::Mat *lcostVol = nullptr;
    cv::Mat *rcostVol = nullptr;
    cv::Mat lValid;
    cv::Mat rValid;

    CVC constructor;
    CVF filter;
    DispSel selector;
    PP postProcessor;

    // Private Methods
    // None
};
