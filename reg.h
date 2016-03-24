
#ifndef REG_H
#define REG_H

#include <opencv2/opencv.hpp>
#include "CommonDefines.h"

// friend classes
struct pyramid_laplacian;			// laplacian pyramid data structure
struct pyramid_laplacian_level;		// laplacian pyramid level
class ROI;							// region of interest
class Affine;						// affine transformation
class LaplacianPyramid;				// laplacian pyramid class

#define NEW 1						// new algorithm

#define SEARCH_X	10				// search region for global correlation
#define SEARCH_Y	10				// search region

// registration parameters
typedef struct {
  int coarseLevel;					/* coarsest resolution level */
  int fineLevel; 					/* finest resolution level */

  int iter[MAX_LEVELS]; 			/* iterations per level */
  MOTION_TYPE motionType[MAX_LEVELS]; /* motion model for each level of processing */

  int searchX, searchY;				// search region
} REG_PARMS;

// registration class
class Reg {
public:

	// constructor
	// image width/height and motion type input
	Reg(int img_width, int img_height, MOTION_TYPE mt=MOTION_TRANSLATION);

	// destructor
	~Reg();

	// set the coarse and fine level for pyramid processing
	void SetLevels(int fineLevel, int coarseLevel);

	// set motion type for registration
	void SetMotionType(MOTION_TYPE mt);

	// set number of iteration for the registration process
	void SetIterations(int iter);

	// set the template for matching
	// ref		:	reference image
	// roi		:	ROI for registration
	bool SetRef(cv::Mat I, ROI &roi);

	// register the input with the template, and output affine motion
	// ins		:	inspection image
	// motion	:	output the affine motion
	bool Register(cv::Mat I, Affine & motion);

	// compute the gaussian/laplacian pyramid for the reference image
	// pyd		:	input the reference image as a pyramid
	// roi		:	ROI for matching
	bool SetRefPyd(pyramid_laplacian &pyd, ROI &roi);

	// set the reference for each pyramid level
	// l		:	pyramid level
	// level	:	number of the level
	// type		:	motion type used in the registration
	bool SetRefLevel(pyramid_laplacian_level &l, ROI &roi, int level, MOTION_TYPE type);

	// registration call
	// l1		:	input inspection image pyramid level
	// l0		:	reference image pyramid level
	// roi		:	ROI for registration
	// level	:	level of the pyramid
	// type		:	motion type used in registration
	// motion	:	affine motion estimated from the registration
	bool RegisterAffine(pyramid_laplacian_level &l1, pyramid_laplacian_level &l0, ROI &roi, int level, MOTION_TYPE type, Affine & motion);

	// delta motion estimation
	void DeltaMotion(short *Ix, short *Iy, short *It, int w, int h, ROI &roi, int levle, MOTION_TYPE type, Affine & dA);
	//void DeltaMotion(short *Ix, short *Iy, short *It, int img_width, int img_height, ROI &roi, int level, MOTION_TYPE type,	float m_Winv[MAX_LEVELS][6][6], float m_WTinv[MAX_LEVELS][2][2], Affine &dA);


	// global correlation search for translation motion
	// l1		:	input pyramid 1
	// I2		:	input pyramid 2
	// roi		:	ROI used in the registration
	// sX		:	search range along x direction
	// sY		:	search range along y direction
	// T		:	initial affine T
	void CorrelationSearch(pyramid_laplacian_level &l1,
						   pyramid_laplacian_level &l2,
					       ROI & roi,
						   int sX, int sY,
						   Affine &T);

	// registration parameters
	REG_PARMS m_parm;

	// image width, height
	int m_width, m_height;

	// ROI for registration
	ROI m_Roi;

//private:

	// motion matrix for each pyramid level
	float m_W[MAX_LEVELS][6][6];

	// motion matrix for each pyramid level, translational motion only
	float m_WT[MAX_LEVELS][2][2];

	// matrix buffer,which is allocated for storing temporary data
	float ** m_A, ** m_AT;

	// inverse motion matrix for each pyramid level
	float m_Winv[MAX_LEVELS][6][6];

	// inverse motion matrix for each pyramid level, translational motion only
	float m_WTinv[MAX_LEVELS][2][2];

	// affine motion estimated from the matching
	Affine m_T;

	// laplacian pyramids used for the inspection and reference images
	LaplacianPyramid *m_laplacian2d[2];
};

// global function
// warp any input image with translational motion
// In			:	input image
// img_width	:	image width
// img_height	:	image height
// mot			:	affine used in the warping process
// roi_ref		:	target ROI in the reference image
// Out			:	output image as the warping result
bool WarpImageTranslation(unsigned char *In,
			              int img_width, int img_height,
						  Affine &mot,			// mot : from Ref -> Inf
						  ROI &roi_ref,			// has to be within the image !
						  unsigned char *Out);

#endif
