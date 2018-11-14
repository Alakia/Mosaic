#pragma once

#include "CommonDefines.h"
#include "Affine.h"
#include <opencv2/opencv.hpp>

class Stablizer;

class StablizerWrapper
{
public:
	StablizerWrapper(MOTION_TYPE type=MOTION_AFFINE);
public:
	~StablizerWrapper(void);

	void Reset();
	void SetMotionType(MOTION_TYPE type);
	void SetIterations(int iter);
	void Stablization(cv::Mat I1, int imgwidth0, int imgheight0, ROI & stab_roi, cv::Mat rgb, int rgbw, int rgbh, cv::Mat I2,int fineLevel=1, int coarseLevel=4);
	void Mosaic(cv::Mat I1, int imgwidth, int imgheight, ROI &stab_roi, cv::Mat rgb4, int in_width, int in_height, cv::Mat rgb3Out, int out_width, int out_height, ROI &roi_ins, float xscale, float yscale, int fineLevel=1, int coarseLevel=4);

private:
	Stablizer *m_Stab;
};
