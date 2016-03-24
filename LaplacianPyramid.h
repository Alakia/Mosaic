// LaplacianPyramid.h: interface for the LaplacianPyramid class.
//
//////////////////////////////////////////////////////////////////////

#ifndef LAPLACIANPYRAMID_H
#define LAPLACIANPYRAMID_H

#include <opencv2/opencv.hpp>

#pragma once

// macro which controls if the edge info is computed or not
#define EDGE_COMPUTATION 0

// maximum level of the pyramid
#define LAPLACIAN_LEVEL	5

// the size of the gaussian kernel
#define GAUSSIAN_MASK_SIZE	5

// 1D gaussian filter
static int gaussian_mask[GAUSSIAN_MASK_SIZE] = {1,4,6,4,1};

// data structure for each pyramid level
struct pyramid_laplacian_level {
	int xsize, ysize;			// image size
	unsigned char *data[2];		// data[0] gaussian pyd, data[1] smoothed gaussian
	short *L;					// buffer
	short *dx;					// x gradient
	short *dy;					// y gradient
	short *dt;					// t gradient

#if EDGE_COMPUTATION
	float *mag;
	unsigned char *edge;		// edge mask
	unsigned char *vedge;		// vertical edge
	unsigned char *hedge;		// horizontal edge
	unsigned char *direc;		// direction of edge
#endif
};

// data structure for laplacian pyramid
struct pyramid_laplacian {
	int nlevels;
	pyramid_laplacian_level *levels;
};

// default threshold for edge computation
// see Canny edge detector
#define DEFAULT_EDGE_HIGH_THRES		6

// class definition
class LaplacianPyramid
{
public:

	// constructor
	// xsize		:	input image width
	// ysize		:	input image height
	// nlevels		:	level of processing
	LaplacianPyramid(int xsize, int ysize, int nlevels = LAPLACIAN_LEVEL);

	// destructor
	virtual ~LaplacianPyramid();

#if EDGE_COMPUTATION
	void DetectEdge(int level, int xstart, int ystart, int xend, int yend);
	void DetectEdge(int level, int xstart, int ystart, int xend, int yend, float hist[4]);
	void ComputeMagAndDirec(int level, int xstart, int ystart, int xend, int yend);
	void FindEdge(int level, int xstart, int ystart, int xend, int yend, float hist[4], float thres);
#endif

	// image size at level 0
	int xsize0, ysize0;

	// pyramid data
	pyramid_laplacian pyd;

	// temporary buffer
	unsigned char *B1;

	// high threshold for double-threshold edge detection
	// see Canny edge detector
	float edge_high_thres;

	// pyramid decomposition
	void analysis2d(cv::Mat I);

	// pyramid synthesis
	void synthesis2d(unsigned char *img);

	// subsample level 1
	void subsample(int l);

	// upsample level l
	void upsample(int l);

	// pyramid decomposition at leve l
	void analysis_level(int l);

	// pyramid synthesis at level l
	void synthesis_level(int l);

	// save all the pyramid components into file, debug function
	void save2file(char *fn);
};

#endif
