/*---------------------------------------------------------------------------
   DispEst.h - Disparity Estimation Class Header
  ---------------------------------------------------------------------------
   Author: Charles Leech
   Email: cl19g10 [at] ecs.soton.ac.uk
   Copyright (c) 2016 Charlie Leech, University of Southampton.
  ---------------------------------------------------------------------------*/
#include "PrimeStereoMatch/ComFunc.h"
#include "PrimeStereoMatch/CVC.h"
#include "PrimeStereoMatch/CVF.h"
#include "PrimeStereoMatch/DispSel.h"
#include "PrimeStereoMatch/PP.h"
#include "PrimeStereoMatch/fastguidedfilter.h"

/**
 * Top-level Disparity Estimation Class 
 */
class PrimeStereoMatch
{
public:
    PrimeStereoMatch(cv::Size imageSize, const int maxDisparity = 64, int numThreads = MAX_CPU_THREADS);

    ~PrimeStereoMatch(void);

    const cv::Mat getLDisp() const {
        return lDisMap;
    }

    const cv::Mat getDisp() const {
        return lDisMap;
    }

    const cv::Mat getRDisp() const {
        return rDisMap;
    }

    const cv::Mat getLValid() const {
        return lValid;
    }

    const cv::Mat getValid() const {
        return lValid;
    }

    const cv::Mat getRValid() const {
        return rValid;
    }

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

    cv::Mat lDisMap;
    cv::Mat rDisMap;

    cv::Mat lValid;
    cv::Mat rValid;

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

    CVC constructor;
    CVF filter;
    DispSel selector;
    PP postProcessor;

    // Private Methods
    // None
};
