// LaplacianPyramid.cpp: implementation of the LaplacianPyramid class.
//
//////////////////////////////////////////////////////////////////////

#include <opencv2/opencv.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include "../include/LaplacianPyramid.h"
#include "math.h"

#define LOW_V		0.4142		// 0.4142 to be 22.5 degree
#define HIGH_V		2.4142		//

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

#define DX_DY		1

// constructor
// input image size and maximum pyramid level
LaplacianPyramid::LaplacianPyramid(int xsize, int ysize, int nlevels)
: xsize0(xsize), ysize0(ysize), edge_high_thres(DEFAULT_EDGE_HIGH_THRES)
{

	pyd.nlevels = nlevels;
	pyd.levels = new pyramid_laplacian_level[nlevels];

	if (!pyd.levels) {
		printf("out of mem");
		exit(1);
	}

	// put some minimal size for level 2
	if (xsize0<80 || ysize0<80 || xsize0%4 != 0 || ysize0%4 != 0) {
		printf("wrong size in LaplacianPyramid");
		exit(1);
	}

	// allocate the momery for each level
	//for (int i=0; i<pyd.nlevels; i++, xsize	>>= 1, ysize >>= 1) {
	for (int i=0; i<pyd.nlevels; i++, xsize	= (xsize+1)>>1, ysize=(ysize+1)>>1) {

		pyd.levels[i].xsize = xsize;
		pyd.levels[i].ysize = ysize;

		int tt = pyd.levels[i].xsize * pyd.levels[i].ysize + 1000;

		for (int j=0;j<2;j++) {
			pyd.levels[i].data[j] = new unsigned char [tt];
			if (!pyd.levels[i].data[j]) {
				printf("out of mem");
				exit(1);
			}

			memset((void *)(pyd.levels[i].data[j]), 0, tt);
		}

		// add dx and dy
		pyd.levels[i].dx = new short [tt];
		pyd.levels[i].dy = new short [tt];
		if (!pyd.levels[i].dx || !pyd.levels[i].dy) {
			printf("out of mem");
			exit(1);
		}
		memset((void *)(pyd.levels[i].dx), 0, tt*sizeof(short));
		memset((void *)(pyd.levels[i].dy), 0, tt*sizeof(short));

		pyd.levels[i].dt = new short [tt];
		if (!pyd.levels[i].dt) {
			printf("out of mem");
			exit(1);
		}
		memset((void *)(pyd.levels[i].dt), 0, tt*sizeof(short));

		pyd.levels[i].L = new short [tt];
		if (!pyd.levels[i].L) {
			printf("out of mem");
			exit(1);
		}
		memset((void *)(pyd.levels[i].L), 0, tt*sizeof(short));

#if EDGE_COMPUTATION
		pyd.levels[i].mag = new float [tt];
		pyd.levels[i].edge = new unsigned char [tt];
		pyd.levels[i].direc = new unsigned char [tt];

		pyd.levels[i].vedge = new unsigned char [tt];
		pyd.levels[i].hedge = new unsigned char [tt];

		if (!pyd.levels[i].mag || !pyd.levels[i].edge || !pyd.levels[i].vedge || !pyd.levels[i].hedge || !pyd.levels[i].direc) {
			printf("out of mem");
			exit(1);
		}

		memset((void *)(pyd.levels[i].mag), 0, tt*sizeof(float));
		memset((void *)(pyd.levels[i].edge), 0, tt);
		memset((void *)(pyd.levels[i].vedge), 0, tt);
		memset((void *)(pyd.levels[i].hedge), 0, tt);
		memset((void *)(pyd.levels[i].direc), 0, tt);
#endif
	}

	B1 = new unsigned char [xsize0*ysize0];

	if (!B1) {
		printf("out of mem");
		exit(1);
	}

	memset((void *)(B1), 0, xsize0*ysize0);
}

// destructor
// free all the momery
LaplacianPyramid::~LaplacianPyramid()
{
	for (int i=0;i<pyd.nlevels;i++) {
		for (int j=0;j<2;j++) {
			if (pyd.levels[i].data[j])
				delete [] pyd.levels[i].data[j];
		}

		if (pyd.levels[i].dx) delete [] pyd.levels[i].dx;
		if (pyd.levels[i].dy) delete [] pyd.levels[i].dy;
		if (pyd.levels[i].dt) delete [] pyd.levels[i].dt;
		if (pyd.levels[i].L) delete [] pyd.levels[i].L;

#if EDGE_COMPUTATION
		if (pyd.levels[i].mag) delete [] pyd.levels[i].mag;
		if (pyd.levels[i].edge) delete [] pyd.levels[i].edge;
		if (pyd.levels[i].vedge) delete [] pyd.levels[i].vedge;
		if (pyd.levels[i].hedge) delete [] pyd.levels[i].hedge;
		if (pyd.levels[i].direc) delete [] pyd.levels[i].direc;
#endif
	}
	if (pyd.levels) delete [] pyd.levels;

	if (B1) delete B1;
}

// main call to do pyramid decomposition
void LaplacianPyramid::analysis2d(cv::Mat I)
{
	//imshow("test", I);cv::waitKey(0);
	// level 0 is the input image
	unsigned char *img = I.data;
	memcpy(pyd.levels[0].data[0],img,xsize0*ysize0*sizeof(unsigned char));

#if EDGE_COMPUTATION
	memset((void *)(pyd.levels[0].edge), 0, pyd.levels[0].xsize*pyd.levels[0].ysize);
	memset((void *)(pyd.levels[0].hedge), 0, pyd.levels[0].xsize*pyd.levels[0].ysize);
	memset((void *)(pyd.levels[0].vedge), 0, pyd.levels[0].xsize*pyd.levels[0].ysize);
#endif

	// NOTE : we only analysis the pyramid up to nlevels-2, instead of the top level nlevels-1
	for (int i=0;i<pyd.nlevels-1;i++) {

		subsample(i);
		analysis_level(i);

#if EDGE_COMPUTATION
		memset((void *)(pyd.levels[i].edge), 0, pyd.levels[i].xsize*pyd.levels[i].ysize);
		memset((void *)(pyd.levels[i].vedge), 0, pyd.levels[i].xsize*pyd.levels[i].ysize);
		memset((void *)(pyd.levels[i].hedge), 0, pyd.levels[i].xsize*pyd.levels[i].ysize);
#endif

	}
}

// pyramid decomposition at each level
void LaplacianPyramid::analysis_level(int l)
{
	// cannot analysis beyond the top and bottom level
	if (l<0 || l>=pyd.nlevels-1) return;

	// upsample level l+1 into l
	upsample(l+1);

	int xsize = pyd.levels[l].xsize;
	int ysize = pyd.levels[l].ysize;

	// compute the difference image to L
	unsigned char *pB1 = pyd.levels[l].data[0];
	unsigned char *pimg = pyd.levels[l].data[1];

	short *ptr = pyd.levels[l].L;

	for (int y=0;y<ysize;y++) {
		for (int x=0;x<xsize;x++) {
			//int t = (*pB1) - (*pimg) + 128;
			ptr[0] = (*pB1) - (*pimg);	// laplacian

			ptr++;	pB1++;	pimg++;
		}
	}

#if DX_DY

	short *pdx = pyd.levels[l].dx + xsize + 1;
	short *pdy = pyd.levels[l].dy + xsize + 1;

	pB1 = pyd.levels[l].data[0] + xsize + 1;

	for (int y=1;y<ysize-1;y++) {
		for (int x=1;x<xsize-1;x++) {

#if 0
			pdx[0] = pimg[1] - pimg[-1];		// Dx
			pdy[0] = pimg[xsize] - pimg[-xsize];	// Dy
#else
			pdx[0] = (pB1[1] - pB1[-1])>>1;		// Dx
			pdy[0] = (pB1[xsize] - pB1[-xsize])>>1;	// Dy
#endif
			pdx ++; pdy ++;
			pB1 ++;
		}
		pB1 += 2;
		pdx += 2;
		pdy += 2;
	}

#endif
}

// subsample level l data[0] into level l+1 data[0]
void LaplacianPyramid::subsample(int l)
{
	if (l<0 || l>=pyd.nlevels-1) return;

	// first smooth image data[0] into data[1]
	// it is done by convolving along x and y direction separately
	int xsize, ysize;
	xsize=pyd.levels[l].xsize;
	ysize=pyd.levels[l].ysize;

	// 1D convolution along x direction with Gaussian kernel
	int xstart = GAUSSIAN_MASK_SIZE/2;
	int xend = xsize-xstart;
	unsigned char *pimg = pyd.levels[l].data[0];
	unsigned char *pB1 = B1 + xstart;

	for (int y=0;y<ysize;y++) {
		for (int x=xstart;x<xend;x++) {
			int t = 0;
			for (int k=0;k<GAUSSIAN_MASK_SIZE;k++) {
				t += gaussian_mask[k] * (*(pimg+k));
			}
			*pB1 = t >> 4;	// MASK_SIZE
			pB1++;	pimg++;
		}
		pB1 += GAUSSIAN_MASK_SIZE - 1;
		pimg += GAUSSIAN_MASK_SIZE - 1;
	}

	// 1D convolution along y direction with Gaussian kernel
	int ystart = xstart;
	int yend = ysize-ystart;
	long offset = xsize*ystart;
	long offset1 = xsize*GAUSSIAN_MASK_SIZE;
	pB1 = B1 + xstart;
	pimg = pyd.levels[l].data[1] + offset + xstart;

	for (int y=ystart;y<yend;y++) {
		for (int x=xstart;x<xend;x++) {
			int t = 0;
			for (int k=0;k<GAUSSIAN_MASK_SIZE;k++) {
				t += gaussian_mask[k] * (*(pB1));
				pB1 += xsize;
			}
			*pimg = t >> 4;		// MASK_SIZE
			pB1 -= offset1;

			pB1++;	pimg++;
		}
		pB1 += GAUSSIAN_MASK_SIZE - 1;
		pimg += GAUSSIAN_MASK_SIZE - 1;
	}

	// fill in the top boundary
	for (int y=ystart-1;y>=0;y--) {
		pimg = pyd.levels[l].data[1] + xstart + y*xsize;
		for (int x=xstart;x<xend;x++) {
			pimg[0] = pimg[xsize];
			pimg ++;
		}
	}

	// fill in the bottom boundary
	for (int y=yend;y<ysize;y++) {
		pimg = pyd.levels[l].data[1] + xstart + y*xsize;
		for (int x=xstart;x<xend;x++) {
			pimg[0] = pimg[-xsize];
			pimg ++;
		}
	}

	// fill in the left and right boundary
	pimg = pyd.levels[l].data[1];
	for (int y=0;y<ysize;y++) {
		for (int x=0;x<xstart;x++) {
			pimg[x]=pimg[xstart];
		}
		for (int x=xend;x<xsize;x++) {
			pimg[x]=pimg[xend-1];
		}
		pimg+=xsize;
	}

	// now sample level l data[1] into level l+1 data[0]
	// ysize = pyd.levels[l].ysize;
	// xsize = pyd.levels[l].xsize;

	unsigned char *pL1 = pyd.levels[l+1].data[0];
	unsigned char *pL0 = pyd.levels[l].data[1];

	for (int y = 0; y < ysize; y += 2, pL0 += xsize*2) {
		pimg = pL0;
		for (int x = 0; x < xsize; x += 2, pL1++, pimg += 2) {
			*pL1 = *pimg;
		}
	}
}

// upsamle level l data[0] to level l-1 data[1] and smooth
void LaplacianPyramid::upsample(int l)
{
	if (l<=0 || l>=pyd.nlevels) return;

	int ysize = pyd.levels[l-1].ysize;
	int xsize = pyd.levels[l-1].xsize;

	unsigned char *pL1 = pyd.levels[l].data[0];
	unsigned char *pL0 = pyd.levels[l-1].data[1];

	// require to be multiple of 2
	for (int y = 0; y < ysize; y += 2) {
		for (int x = 0; x < xsize; x += 2) {
			pL0[0] = pL1[0];
			pL0[1] = pL1[0];
			pL0[xsize] = pL1[0];
			pL0[xsize+1]=pL1[0];
			pL1++;
			pL0+=2;
		}
		pL0+=xsize;
	}

	// smoothing data[1]
	// it is done by convolving along x and y direction separately

	int xstart = GAUSSIAN_MASK_SIZE/2;
	int xend = xsize-xstart;
	unsigned char *pimg = pyd.levels[l-1].data[1];
	unsigned char *pB1 = B1 + xstart;

	for (int y=0;y<ysize;y++) {
		for (int x=xstart;x<xend;x++) {
			int t = 0;
			for (int k=0;k<GAUSSIAN_MASK_SIZE;k++) {
				t += gaussian_mask[k] * (*(pimg+k));
			}
			*pB1 = t >> 4;	// MASK_SIZE
			pB1++;	pimg++;
		}
		pB1 += GAUSSIAN_MASK_SIZE - 1;
		pimg += GAUSSIAN_MASK_SIZE - 1;
	}

	int ystart = xstart;
	int yend = ysize-ystart;
	long offset = xsize*ystart;
	long offset1 = xsize*GAUSSIAN_MASK_SIZE;
	pB1 = B1 + xstart;
	pimg = pyd.levels[l-1].data[1] + offset + xstart;

	for (int y=ystart;y<yend;y++) {
		for (int x=xstart;x<xend;x++) {
			int t = 0;
			for (int k=0;k<GAUSSIAN_MASK_SIZE;k++) {
				t += gaussian_mask[k] * (*(pB1));
				pB1 += xsize;
			}
			*pimg = t >> 4;		// MASK_SIZE
			pB1 -= offset1;

			pB1++;	pimg++;
		}
		pB1 += GAUSSIAN_MASK_SIZE - 1;
		pimg += GAUSSIAN_MASK_SIZE - 1;
	}

	// padding boundaries
	for (int y=ystart-1;y>=0;y--) {
		pimg = pyd.levels[l-1].data[1] + xstart + y*xsize;
		for (int x=xstart;x<xend;x++) {
			pimg[0] = pimg[xsize];
			pimg ++;
		}
	}

	for (int y=yend;y<ysize;y++) {
		pimg = pyd.levels[l-1].data[1] + xstart + y*xsize;
		for (int x=xstart;x<xend;x++) {
			pimg[0] = pimg[-xsize];
			pimg ++;
		}
	}

	pimg = pyd.levels[l-1].data[1];
	for (int y=0;y<ysize;y++) {
		for (int x=0;x<xstart;x++) {
			pimg[x]=pimg[xstart];
		}
		for (int x=xend;x<xsize;x++) {
			pimg[x]=pimg[xend-1];
		}
		pimg+=xsize;
	}
}

// synthesize the level l image with the following formula
// data[1] + L = data[0]
void LaplacianPyramid::synthesis_level(int l)
{
	// right now only 2 levels
	if (l<0 || l>=pyd.nlevels-1) return;

	// add the difference image L to data[0]
	unsigned char *pB1 = pyd.levels[l].data[0];
	unsigned char *pimg = pyd.levels[l].data[1];

	short *ptr = pyd.levels[l].L;

	int xsize = pyd.levels[l].xsize;
	int ysize = pyd.levels[l].ysize;

	for (int y=0;y<ysize;y++) {
		for (int x=0;x<xsize;x++) {
			int g = ptr[0] + pimg[0];	// laplacian
			if (g>255) g=255;
			if (g<0) g=0;
			pB1[0]=g;

			ptr++;	pB1++;	pimg++;
		}
	}
}

// synthesize the original image from the pyramid
void LaplacianPyramid::synthesis2d(unsigned char *img)
{
	for (int i=pyd.nlevels-1;i>0;i--) {
		upsample(i);
		synthesis_level(i-1);
	}

	memcpy(img, pyd.levels[0].data[0], xsize0*ysize0*sizeof(unsigned char));
}

// save all the relevant components in the pyramid processing into files
// debug purpose
void LaplacianPyramid::save2file(char *fn)
{
	FILE *fp;
	char fn1[100];

	for (int i=0;i<pyd.nlevels;i++) {

		sprintf(fn1,"%s_%d_data0.txt",fn,i);
		fp = fopen(fn1,"wt");

		unsigned char *ptr = pyd.levels[i].data[0];

		for (int y=0;y<pyd.levels[i].ysize;y++) {
			for (int x=0;x<pyd.levels[i].xsize;x++) {
				fprintf(fp,"%d ", ptr[0]);
				ptr++;
			}
			fprintf(fp,"\n");
		}
		fclose(fp);

		sprintf(fn1,"%s_%d_dx.txt",fn,i);
		fp = fopen(fn1,"wt");

		short *ptr1 = pyd.levels[i].dx;

		for (int y=0;y<pyd.levels[i].ysize;y++) {
			for (int x=0;x<pyd.levels[i].xsize;x++) {
				fprintf(fp,"%d ", ptr1[0]);
				ptr1++;
			}
			fprintf(fp,"\n");
		}
		fclose(fp);

	}
}


#if EDGE_COMPUTATION
void LaplacianPyramid::DetectEdge(int l, int xstart, int ystart, int xend, int yend)
{
	int offset = pyd.levels[l].xsize*ystart + xstart;

	float *pf = pyd.levels[l].mag + offset;
	short *pdx=pyd.levels[l].dx + offset;
	short *pdy=pyd.levels[l].dy + offset;

	/*
	for (int i=0;i<pyd.levels[l].ysize;i++)
		for (int j=0;j<pyd.levels[l].xsize;j++) {
			pf[0]=sqrt((double)(pdx[0])*(double)(pdx[0]) + (double)(pdy[0])*(double)(pdy[0]));
			pf ++;
			pdx ++;
			pdy ++;
		}
	*/

	int jump = pyd.levels[l].xsize - (xend-xstart+1);

	for (int i=ystart;i<=yend;i++) {
		for (int j=xstart;j<=xend;j++) {
			pf[0]=sqrt((double)(pdx[0])*(double)(pdx[0]) + (double)(pdy[0])*(double)(pdy[0]));
			pf ++;
			pdx ++;
			pdy ++;
		}
		pf += jump;
		pdx += jump;
		pdy += jump;
	}

	float * pmag = pyd.levels[l].mag + offset + pyd.levels[l].xsize + 1;
	unsigned char * pedge = pyd.levels[l].edge + offset + pyd.levels[l].xsize + 1;

	unsigned char * pvedge = pyd.levels[l].vedge + offset + pyd.levels[l].xsize + 1;
	unsigned char * phedge = pyd.levels[l].hedge + offset + pyd.levels[l].xsize + 1;

	pdx = pyd.levels[l].dx + offset + pyd.levels[l].xsize + 1;
	pdy = pyd.levels[l].dy + offset + pyd.levels[l].xsize + 1;

	jump = jump + 2;

	int xSize = pyd.levels[l].xsize;

	for (int i=ystart+1;i<yend;i++) {
		for (int j=xstart+1;j<xend;j++) {

			bool flag = false;
			int direc;

			if (pdx[0]==0 && pdy[0]==0)
				goto stop;						// not an edge
			else if (pdx[0]==0)
				direc = 2;
			else if (pdy[0]==0)
				direc = 0;
			else {
				float r = (float)pdx[0]/(float)pdy[0];
				if (r<0) r=-r;

				if (r<LOW_V) {
					direc = 2;
				} else if (r>HIGH_V) {	// 67.5 degrees
				//} else if (r>1.3) {
					direc = 0;
				} else if(pdx[0]>0 && pdy[0]>0 || pdx[0]<0 && pdy[0]<0) {
					direc = 3;
				} else {
					direc = 1;
				}
			}

			switch (direc) {
				case 0:
					if (pmag[0]>pmag[1] && pmag[0]>pmag[-1]) {
						if (pedge[-xSize-1] || pedge[-xSize] || pedge[-xSize+1] ||
							pedge[xSize-1] || pedge[xSize] || pedge[xSize+1])
							flag = true;
					} else
						goto stop;

					break;
				case 1:
					if (pmag[0]>pmag[-xSize+1] && pmag[0]>pmag[xSize-1]) {
						if (pedge[-xSize-1] || pedge[-xSize] || pedge[-1] ||
							pedge[xSize] || pedge[xSize+1] || pedge[1])
							flag = true;
					} else
						goto stop;

					break;
				case 2:
					if (pmag[0]>pmag[-xSize] && pmag[0]>pmag[xSize]) {
						if (pedge[-xSize-1] || pedge[-1] || pedge[-xSize+1] ||
							pedge[xSize-1] || pedge[1] || pedge[xSize+1])
							flag = true;
					} else
						goto stop;

					break;
				case 3:
					if (pmag[0]>pmag[-xSize-1] && pmag[0]>pmag[xSize+1]) {
						if (pedge[-xSize] || pedge[-1] || pedge[-xSize+1] ||
							pedge[xSize-1] || pedge[xSize] || pedge[1])
							flag = true;
					} else
						goto stop;

					break;
				default:
					break;
			}

			if (flag) {
				if (pmag[0]>10) {
					pedge[0] = 1;

					if (direc==0)				// vertical edge
						pvedge[0] = 1;

					if (direc==2)				// vertical edge
						phedge[0] = 1;
				}
			} else {
				if (pmag[0]>10) {
					pedge[0] = 1;

					if (direc==0)				// vertical edge
						pvedge[0] = 1;

					if (direc==2)				// vertical edge
						phedge[0] = 1;
				}
			}

			stop:
			pedge ++;
			pvedge ++;
			phedge ++;
			pmag ++;
			pdx ++;
			pdy ++;
		}
		pedge += jump;
		pvedge += jump;
		phedge += jump;
		pmag += jump;
		pdx += jump;
		pdy += jump;
	}
}

void LaplacianPyramid::DetectEdge(int l, int xstart, int ystart, int xend, int yend, float hist[4])
{
	ComputeMagAndDirec(l, xstart, ystart, xend, yend);
	FindEdge(l, xstart, ystart, xend, yend, hist, edge_high_thres);
}

void LaplacianPyramid::FindEdge(int l, int xstart, int ystart, int xend, int yend, float hist[4], float thres)
{
	long offset = pyd.levels[l].xsize*ystart + xstart;

	int jump = pyd.levels[l].xsize - (xend-xstart+1);

	float * pmag = pyd.levels[l].mag + offset + pyd.levels[l].xsize + 1;
	unsigned char * pedge = pyd.levels[l].edge + offset + pyd.levels[l].xsize + 1;
	unsigned char * pvedge = pyd.levels[l].vedge + offset + pyd.levels[l].xsize + 1;
	unsigned char * phedge = pyd.levels[l].hedge + offset + pyd.levels[l].xsize + 1;
	unsigned char * pdirec = pyd.levels[l].direc + offset + pyd.levels[l].xsize + 1;

	jump = jump + 2;

	int xSize = pyd.levels[l].xsize;

	for (int i=0;i<4;i++)
		hist[i] = 0;

	for (int i=ystart+1;i<yend;i++) { // DJH: Why are all these boundaries + or - 1?
		for (int j=xstart+1;j<xend;j++) {

			bool flag = false;

			switch (pdirec[0]) {
				case 0:
					if (pmag[0]>=pmag[1] && pmag[0]>=pmag[-1]) {
						if (pedge[-xSize-1] || pedge[-xSize] || pedge[-xSize+1] ||
							pedge[xSize-1] || pedge[xSize] || pedge[xSize+1])
							flag = true;
					} else
						goto stop;

					break;
				case 1:
					if (pmag[0]>=pmag[-xSize+1] && pmag[0]>=pmag[xSize-1]) {
						if (pedge[-xSize-1] || pedge[-xSize] || pedge[-1] ||
							pedge[xSize] || pedge[xSize+1] || pedge[1])
							flag = true;
					} else
						goto stop;

					break;
				case 2:
					if (pmag[0]>=pmag[-xSize] && pmag[0]>=pmag[xSize]) {
						if (pedge[-xSize-1] || pedge[-1] || pedge[-xSize+1] ||
							pedge[xSize-1] || pedge[1] || pedge[xSize+1])
							flag = true;
					} else
						goto stop;

					break;
				case 3:
					if (pmag[0]>=pmag[-xSize-1] && pmag[0]>=pmag[xSize+1]) {
						if (pedge[-xSize] || pedge[-1] || pedge[-xSize+1] ||
							pedge[xSize-1] || pedge[xSize] || pedge[1])
							flag = true;
					} else
						goto stop;

					break;
				default:
					goto stop;
					break;
			}

			if (flag) {
				if (pmag[0]>thres/2.0f) {
					pedge[0] = 1;
					hist[pdirec[0]] ++;

					if (pdirec[0]==0)				// vertical edge
						pvedge[0] = 1;
					if (pdirec[0]==2)				// horizontal edge
						phedge[0] = 1;
				}
			} else {
				if (pmag[0]>edge_high_thres) {
					pedge[0] = 1;

					hist[pdirec[0]] ++;

					if (pdirec[0]==0)				// vertical edge
						pvedge[0] = 1;
					if (pdirec[0]==2)				// horizontal edge
						phedge[0] = 1;
				}
			}

			stop: if (pedge[0]==0) pdirec[0]=255;

			pedge ++;
			pvedge ++;
			phedge ++;
			pmag ++;
			pdirec ++;
		}
		pedge += jump;
		pvedge += jump;
		phedge += jump;
		pmag += jump;
		pdirec += jump;
	}

	float total_edge = 0;

	for (int i=0;i<4;i++)
		total_edge += hist[i];

	if (total_edge>0) {
		for (int i=0;i<4;i++)
			hist[i] = hist[i]/total_edge;
	}
}

void LaplacianPyramid::ComputeMagAndDirec(int l, int xstart, int ystart, int xend, int yend)
{
	long offset = pyd.levels[l].xsize*ystart + xstart;

	float *pf = pyd.levels[l].mag + offset;
	short *pdx=pyd.levels[l].dx + offset;
	short *pdy=pyd.levels[l].dy + offset;

	int jump = pyd.levels[l].xsize - (xend-xstart+1);

	for (int i=ystart;i<=yend;i++) {
		for (int j=xstart;j<=xend;j++) {
			pf[0]=sqrt((double)(pdx[0])*(double)(pdx[0]) + (double)(pdy[0])*(double)(pdy[0])); // Purify UMR [fix border via {xstart, ystart, etc}]
			pf ++;
			pdx ++;
			pdy ++;
		}
		pf += jump;
		pdx += jump;
		pdy += jump;
	}

	float * pmag = pyd.levels[l].mag + offset + pyd.levels[l].xsize + 1;
	unsigned char * pdirec = pyd.levels[l].direc + offset + pyd.levels[l].xsize + 1;

	pdx = pyd.levels[l].dx + offset + pyd.levels[l].xsize + 1;
	pdy = pyd.levels[l].dy + offset + pyd.levels[l].xsize + 1;

	jump = jump + 2;

	int xSize = pyd.levels[l].xsize;

	for (int i=ystart+1;i<yend;i++) {
		for (int j=xstart+1;j<xend;j++) {

			unsigned char dir;

			if (pdx[0]==0 && pdy[0]==0)
				dir = 255;						// not an edge
			else if (pdx[0]==0)
				dir = 2;
			else if (pdy[0]==0)
				dir = 0;
			else {
				float r = (float)pdx[0]/(float)pdy[0];
				if (r<0) r=-r;

				if (r<LOW_V) {
					dir = 2;
				} else if (r>HIGH_V) {
					dir = 0;
				} else if(pdx[0]>0 && pdy[0]>0 || pdx[0]<0 && pdy[0]<0) {
					dir = 3;
				} else {
					dir = 1;
				}
			}

			pdirec[0] = dir;

			pdirec ++;
			pdx ++;
			pdy ++;
		}
		pdirec += jump;
		pdx += jump;
		pdy += jump;
	}
}
#endif
