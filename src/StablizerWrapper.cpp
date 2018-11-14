#include "../include/StablizerWrapper.h"
#include "../include/Stablizer.h"
#include <opencv2/opencv.hpp>

// wrapper class

// constructor
// allocate the registration member
StablizerWrapper::StablizerWrapper(MOTION_TYPE type)
{
	m_Stab = new Stablizer(type);
}

// destructor
StablizerWrapper::~StablizerWrapper(void)
{
	delete m_Stab;
}

// reset the registration member
void StablizerWrapper::Reset()
{
	m_Stab->Reset();
}

// set the motion type used in registration
void StablizerWrapper::SetMotionType(MOTION_TYPE type)
{
	m_Stab->SetMotionType(type);
}

// set the number of iteration in the registration
void StablizerWrapper::SetIterations(int iter)
{
	m_Stab->SetIterations(iter);
}

// stabilization call
void StablizerWrapper::Stablization(cv::Mat I1, int imgwidth0, int imgheight0, ROI &stab_roi, cv::Mat rgb, int rgbw, int rgbh, cv::Mat I2, int fineLevel, int coarseLevel)
{
	m_Stab->Stablization(I1,imgwidth0, imgheight0,stab_roi,rgb,rgbw,rgbh,I2,fineLevel,coarseLevel);
}

// mosaic function call
void StablizerWrapper::Mosaic(cv::Mat I1, int imgwidth, int imgheight, ROI &stab_roi, cv::Mat rgb4, int in_width, int in_height, cv::Mat rgb3Out, int out_width, int out_height, ROI &roi_ins, float xscale, float yscale, int fineLevel, int coarseLevel)
{
	m_Stab->Mosaic(I1, imgwidth,imgheight,stab_roi,rgb4,in_width, in_height, rgb3Out, out_width, out_height, roi_ins, xscale, yscale, fineLevel, coarseLevel);
}
