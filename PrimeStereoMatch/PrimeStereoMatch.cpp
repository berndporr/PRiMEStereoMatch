/*---------------------------------------------------------------------------
   DispEst.cpp - Disparity Estimation Class
  ---------------------------------------------------------------------------
   Author: Charles Leech
   Email: cl19g10 [at] ecs.soton.ac.uk
   Copyright (c) 2016 Charlie Leech, University of Southampton.
  ---------------------------------------------------------------------------*/
#include "PrimeStereoMatch.h"

PrimeStereoMatch::PrimeStereoMatch(int height, int width, const int maxDisparity, int numThreads)
    : hei(height), wid(width), maxDis(maxDisparity), threads(numThreads)
{
    lcostVol = new cv::Mat[maxDis];
    rcostVol = new cv::Mat[maxDis];
    for (int i = 0; i < maxDis; ++i)
    {
        lcostVol[i] = cv::Mat::zeros(hei, wid, CV_32FC1);
        rcostVol[i] = cv::Mat::zeros(hei, wid, CV_32FC1);
    }

    lDisMap = cv::Mat::zeros(hei, wid, CV_8UC1);
    rDisMap = cv::Mat::zeros(hei, wid, CV_8UC1);
    lValid = cv::Mat::zeros(hei, wid, CV_8UC1);
    rValid = cv::Mat::zeros(hei, wid, CV_8UC1);
}

PrimeStereoMatch::~PrimeStereoMatch(void)
{
    delete[] lcostVol;
    delete[] rcostVol;
}

int PrimeStereoMatch::setInputImages(cv::Mat leftImg, cv::Mat rightImg)
{
    if (leftImg.type() != rightImg.type()) return 1;
    lImg = leftImg;
    rImg = rightImg;
    if ((lImg.type() & CV_MAT_DEPTH_MASK) != CV_32F)
    {
	lImg.convertTo(lImg, CV_32F, 1 / 255.0f);
    }
    if ((rImg.type() & CV_MAT_DEPTH_MASK) != CV_32F)
    {
	rImg.convertTo(rImg, CV_32F, 1 / 255.0f);
    }
    return 0;
}

int PrimeStereoMatch::setThreads(unsigned int newThreads)
{
    if (newThreads > MAX_CPU_THREADS)
        return -1;

    threads = newThreads;
    return 0;
}

// ##########################
// # Cost Volume Construction
// ##########################
int PrimeStereoMatch::CostConst()
{
    int ret_val = 0;

    ret_val = constructor.preprocess(lImg, lGrdX);
    if (ret_val)
        return ret_val;
    ret_val = constructor.preprocess(rImg, rGrdX);
    if (ret_val)
        return ret_val;

// Build Cost Volume
#pragma omp parallel for
    for (int d = 0; d < maxDis; ++d)
    {
        constructor.buildCV_left(lImg, rImg, lGrdX, rGrdX, d, lcostVol[d]);
    }
#pragma omp parallel for
    for (int d = 0; d < maxDis; ++d)
    {
        constructor.buildCV_right(rImg, lImg, rGrdX, lGrdX, d, rcostVol[d]);
    }
    return 0;
}

int PrimeStereoMatch::CostConst_CPU()
{
    // Set up threads and thread attributes
    void *status;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    std::vector<pthread_t> BCV_threads;
    BCV_threads.resize(maxDis);
    std::vector<buildCV_TD> buildCV_TD_Array;
    buildCV_TD_Array.resize(maxDis);

    constructor.preprocess(lImg, lGrdX);
    constructor.preprocess(rImg, rGrdX);

    for (int level = 0; level <= maxDis / threads; ++level)
    {
        // Handle remainder if threads is not power of 2.
        int block_size = (level < maxDis / threads) ? threads : (maxDis % threads);

        for (int iter = 0; iter < block_size; ++iter)
        {
            int d = level * threads + iter;
            buildCV_TD_Array[d] = {&lImg, &rImg, &lGrdX, &rGrdX, d, &lcostVol[d]};
            pthread_create(&BCV_threads[d], &attr, CVC::buildCV_left_thread, (void *)&buildCV_TD_Array[d]);
        }
        for (int iter = 0; iter < block_size; ++iter)
        {
            int d = level * threads + iter;
            pthread_join(BCV_threads[d], &status);
        }
    }
    for (int level = 0; level <= maxDis / threads; ++level)
    {
        // Handle remainder if threads is not power of 2.
        int block_size = (level < maxDis / threads) ? threads : (maxDis % threads);

        for (int iter = 0; iter < block_size; ++iter)
        {
            int d = level * threads + iter;
            buildCV_TD_Array[d] = {&rImg, &lImg, &rGrdX, &lGrdX, d, &rcostVol[d]};
            pthread_create(&BCV_threads[d], &attr, CVC::buildCV_right_thread, (void *)&buildCV_TD_Array[d]);
        }
        for (int iter = 0; iter < block_size; ++iter)
        {
            int d = level * threads + iter;
            pthread_join(BCV_threads[d], &status);
        }
    }
    return 0;
}

// #######################
// # Cost Volume Filtering
// #######################
int PrimeStereoMatch::CostFilter_FGF()
{
    FastGuidedFilter fgf_left(lImg, GIF_R_WIN, GIF_EPS, subsample_rate);
    FastGuidedFilter fgf_right(rImg, GIF_R_WIN, GIF_EPS, subsample_rate);

#pragma omp parallel for
    for (int d = 0; d < maxDis; ++d)
    {
        lcostVol[d] = fgf_left.filter(lcostVol[d]);
    }

#pragma omp parallel for
    for (int d = 0; d < maxDis; ++d)
    {
        rcostVol[d] = fgf_right.filter(rcostVol[d]);
    }
    return 0;
}

int PrimeStereoMatch::DispSelect_CPU()
{
    // printf("Left Selection...\n");
    selector.CVSelect(lcostVol, maxDis, lDisMap);
    // selector->CVSelect_thread(lcostVol, maxDis, lDisMap, threads);

    // printf("Right Selection...\n");
    selector.CVSelect(rcostVol, maxDis, rDisMap);
    // selector->CVSelect_thread(rcostVol, maxDis, rDisMap, threads);
    return 0;
}

int PrimeStereoMatch::PostProcess_CPU()
{
    // printf("Post Processing Underway...\n");
    postProcessor.processDM(lImg, rImg, lDisMap, rDisMap, lValid, rValid, maxDis, threads);
    // printf("Post Processing Complete\n");
    return 0;
}

void PrimeStereoMatch::process()
{
    CostConst();
    CostFilter_FGF();
    DispSelect_CPU();
    PostProcess_CPU();
}
