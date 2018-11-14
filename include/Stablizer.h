
#ifndef STABLIZER_H
#define STABLIZER_H

#include "CommonDefines.h"
#include <opencv2/opencv.hpp>

// the following macros are defined to set the ROI
#define MAX_X	100
#define MAX_Y	80
#define MIN_X	50
#define MIN_Y	40

class ROI;
class Affine;
class Reg;

// stablization class
class Stablizer
{
public:

	// constructor
	// type				:	motion type in stabilization
	Stablizer(MOTION_TYPE type=MOTION_AFFINE);

	// destructor
	~Stablizer();

	// reset the stabilizer
	void Reset();

	// set the motion type
	void SetMotionType(MOTION_TYPE type);

	// set the number of iterations
	void SetIterations(int iter);

	// stabilization call
	// input I1		:	input grayscale image
	// imgwidth0	:	input image width
	// imgheight0	:	input image height
	// stab_roi		:	ROI used for stabilization
	// rgb			:	input rgb image for stabilization
	// rgbw			:	input rgb image width
	// rgbh			:	input rgb image height
	// I2			:	stabilized output image
	// fineLevel	:	fine level of processing
	// coarseLevel	:	coarse level of processing
	void Stablization(cv::Mat I1, int imgwidth0, int imgheight0, ROI & stab_roi, cv::Mat rgb, int rgbw, int rgbh, cv::Mat I2,int fineLevel=1, int coarseLevel=4);

	// mosaic function call
	// I1			:	input grayscale image
	// imgwidth		:	input image width
	// imgheight	:	input image height
	// stab_roi		:	ROI used for stabilization
	// rgb4			:	input rgb4 image for mosaic
	// in_width		:	input rgb4 image width
	// in_height	:	input rgb4 image height
	// rgb3Out		:	mosaic rgb3 output image
	// out_width	:	mosaic image width
	// out_height	:	mosaic image height
	// roi_ins		:	useful portion of the inspection image to generate the mosaic
	// xscale		:	x scale factor for mosaic, for example, if xscale=0.5, the mosaic is generated with half the width of the inspection image
	// yscale		:	y scale factor for mosaic
	// fineLevel	:	fine level of processing
	// coarseLevel	:	coarse level of processing
	void Mosaic(cv::Mat I1, int imgwidth, int imgheight, ROI &stab_roi, cv::Mat rgb4, int in_width, int in_height, cv::Mat rgb3Out, int out_width, int out_height, ROI &roi_ins, float xscale, float yscale, int fineLevel=1, int coarseLevel=4);

	// warp input image to output mosaic
	// roi_ref defines the region to warp in the reference frame
	// rgb4			:	input rgb4 color image for mosaic
	// in_width		:	input image width
	// in_height	:	input image height
	// rgb3Out		:	output rgb3 mosaic image
	// out_width	:	mosaic image width
	// out_height	:	mosaic image height
	// roi_ins		:	ROI used in the input image for mosaic
	// mot			:	estimated affine motion used in warping
	// xscale		:	scale factor in x
	// yscale		:	scale factor in y
	bool WarpRGBAffineMosaic(cv::Mat rgb4, int in_width, int in_height, cv::Mat rgb3Out, int out_width, int out_height, ROI &roi_ins, Affine& mot, float xscale, float yscale);

	// alpha for attenuating
	float m_alpha;

	// motion type for registration
	MOTION_TYPE m_type;

	//float reducingScale;
	//int reduced_width;
	//int reduced_height;
	//void ComputeReducedSize(int xsize, int ysize);
	Affine m_current_T;				// transformation from last templ update
	Affine m_current_T_save;		// saved copy of current transformation
	Affine m_last_templ_update_T;	// transformation up to last templ update
	Affine m_global_T;				// global transformation from ref to current frame

private:

	// image warping with translation motion
	// rgb4			:	input rgb4 image
	// w			:	input image width
	// h			:	input image height
	// rgb3Out		:	output rgb3 image
	// ref			:	ROI in the reference image used in warping
	// tx			:	tarnslation x
	// ty			:	translation y
	void WarpRGBTranslation(cv::Mat rgb4, int w, int h, cv::Mat rgb3Out, ROI &ref, float tx, float ty);

	// image warping with affine motion
	// rgb4			:	input rgb4 image
	// w			:	input image width
	// h			:	input image height
	// rgb3Out		:	output rgb3 mosaic image
	// ref			:	ROI in the reference image used for warping
	// affine		:	affine motion for warping
	void WarpRGBAffine(cv::Mat rgb4, int w, int h, cv::Mat rgb3Out, ROI &ref, Affine &affine);

	// check if current motion is beyond preset threshold
	// m			:	affine motion
	// xc			:	affine origin x
	// yc			:	affine origin y
	// maxx			:	max x translation
	// maxy			:	max y translation
	bool LargeMotion(Affine m, int xc, int yc, int maxx, int maxy);

	// check if current motion is small
	bool SmallMotion(Affine m);

	// registration class member
	Reg *m_Reg;
};

#endif
