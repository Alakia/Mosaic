
#include "../include/Affine.h"
#include "../include/LaplacianPyramid.h"

#include "../include/reg.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../include/gaussj.h"

// global delta motion calculation function
//void DeltaMotion(short *Ix, short *Iy, short *It, int img_width, int img_height, ROI &roi, int level, MOTION_TYPE type,	float m_Winv[MAX_LEVELS][6][6], float m_WTinv[MAX_LEVELS][2][2], Affine &dA);
//void DeltaMotion(short *Ix, short *Iy, short *It, int img_width, int img_height, ROI &roi, int level, MOTION_TYPE type, Affine &dA);


// warp image with affine
// the input and output images have different sizes
// In			:		Input image
// in_width		:		input image width
// in_height	:		input image height
// mot			:		affine motion used to warping
// roi_out		:		output ROI
// Out			:		output image
// out_width	:		output image width
// out_height	:		output image height
bool WarpImageAffine2(unsigned char *In,
	int in_width, int in_height,
	Affine &mot,						// mot : from ref -> Ins
	ROI &roi_out,
	unsigned char *Out,
	int out_width, int out_height)
{
	int i,j, intx, inty;
	unsigned char *outRow, *outCol;

	float xc, yc;
	mot.getOrigin(xc, yc);

	if (xc!=0 || yc!=0) {
		printf("wrong origin in affine");
		return false;
	}

	// loop through the reference roi, to process each pixel in the ROI
	for(j = roi_out.yStart,outRow=Out+j*out_width;j<=roi_out.yEnd;j++,outRow+=out_width) {

		float fj = j;

		// apply the motion to compute the corresponding pixel in the inspection image
		float trsx_fj = mot.A[0][1]*fj + mot.A[0][2];
		float trsy_fj = mot.A[1][1]*fj + mot.A[1][2];

		// loop through each row
		for(i=roi_out.xStart,outCol=outRow+i;i<=roi_out.xEnd;i++,outCol++) {
			float fi,yy,xx;

			fi = (float)i;

			// apply the motion to compute the corresponding pixel in the inspection image
			yy = mot.A[1][0]*fi + trsy_fj;
			xx = mot.A[0][0]*fi + trsx_fj;

			// take the closeset pixel to (xx, yy)
			inty = (int) QFTOL(yy);
			intx = (int) QFTOL(xx);

			// consider those pixels located inside the input image region
			// note : the following test also gurantees that (intx,inty) are both positive numbers
			if (((unsigned int) intx) < in_width-1 && ((unsigned int) inty) <in_height-1) {
				float xfrac = xx - (float)(intx);
				float yfrac = yy - (float)(inty);

				unsigned char *ptr1= In+inty*in_width+intx;
				unsigned char *ptr2= ptr1 + in_width;

				// bilinear interpolation
				float y1val= ((float)ptr1[0]) + xfrac*((float)ptr1[1]-(float)ptr1[0]);
				float y2val= ((float)ptr2[0]) + xfrac*((float)ptr2[1]-(float)ptr2[0]);
				float val=y1val + yfrac*(y2val-y1val);

				if (val>255) val = 255;

				outCol[0]= (unsigned char)(QFTOL(val+0.5));
			}
			else {
				// fill in black if the corresponding pixel is outside the input image
				outCol[0]=0;
			}
		}
	}

	return true;
}

// warp image with affine
// the input and output images have the same size
// In			:		Input image
// img_width	:		input image width
// img_height	:		input image height
// mot			:		affine motion used to warping
// roi_ref		:		output ROI
// Out			:		output image
bool WarpImageAffine(unsigned char *In,
	int img_width, int img_height,
	Affine &mot,						// mot : from ref -> Ins
	ROI &roi_ref,
	unsigned char *Out)
{
	int i,j, intx, inty;
	unsigned char *outRow, *outCol;

	float xc, yc;
	mot.getOrigin(xc, yc);

	if (xc!=0 || yc!=0) {
		printf("wrong origin in affine");
		return false;
	}

	// loop through the reference roi, to process each pixel in the ROI
	for(j = roi_ref.yStart,outRow=Out+j*img_width;j<=roi_ref.yEnd;j++,outRow+=img_width) {

		float fj = j;

		// apply the affine motion to compute the corresponding pixel in the input image
		// this part depends only on fj, therefore it is computed once for each row
		float trsx_fj = mot.A[0][1]*fj + mot.A[0][2];
		float trsy_fj = mot.A[1][1]*fj + mot.A[1][2];

		// loop through each row
		for(i=roi_ref.xStart,outCol=outRow+i;i<=roi_ref.xEnd;i++,outCol++) {
			float fi,yy,xx;

			fi = (float)i;

			// apply the affine motion to compute the corresponding pixel in the input image
			yy = mot.A[1][0]*fi + trsy_fj;
			xx = mot.A[0][0]*fi + trsx_fj;

			// take the closest integter of (xx,yy)
			inty = (int) QFTOL(yy);
			intx = (int) QFTOL(xx);

			// only the corresponding pixels located within the input image is processed
			if (((unsigned int) intx) < img_width-1 && ((unsigned int) inty) <img_height-1) {
				float xfrac = xx - (float)(intx);
				float yfrac = yy - (float)(inty);

				unsigned char *ptr1= In+inty*img_width+intx;
				unsigned char *ptr2= ptr1 + img_width;

				// bilinear interpolation
				float y1val= ((float)ptr1[0]) + xfrac*((float)ptr1[1]-(float)ptr1[0]);
				float y2val= ((float)ptr2[0]) + xfrac*((float)ptr2[1]-(float)ptr2[0]);
				float val=y1val + yfrac*(y2val-y1val);

				if (val>255) val = 255;

				outCol[0]= (unsigned char)(QFTOL(val+0.5));
			}
			else {
				// fill in black if the corresponding pixel is outside the input image
				outCol[0]=0;
			}
		}
	}

	return true;
}


// warp image with translation only
// the input and output images have the same size
// In			:		Input image
// img_width	:		input image width
// img_height	:		input image height
// mot			:		affine motion used to warping
// roi_ref		:		output ROI
// Out			:		output image
bool WarpImageTranslation(unsigned char *In,	// inspection data
	int img_width, int img_height,
	Affine &mot,			// mot : from ref -> Inf
	ROI &roi_ref,			// has to be within the image !
	unsigned char *Out)
{
	roi_ref = roi_ref.crop(img_width, img_height);
	float dx, dy, ddx, ddy;

	int xs1, xs2, ys1, ys2, xe1, xe2, ye1, ye2;

	float tx = mot.A[0][2];
	float ty = mot.A[1][2];

	// by classifying the (tx,ty), we exploit the efficiency

	if (tx<0) {						// moving to left

		dx=QFTOLRND(tx)-tx;			// e.g. -1.2 --> dx = 0.2

		if (dx<1e-4) {							// close to an integer, e.g. 1.0002
			int tmp = QFTOLRND(tx);
			xs2 = roi_ref.xStart+tmp;
			if (xs2<0) xs2 = 0;					// make sure it is in image
			xs1=xs2-tmp;						//
			xe1=roi_ref.xEnd;					//
			if (xe1>=img_width)
				xe1=img_width-1;
			dx=0;
			ddx=1;
		} else if (1-dx<1e-4) {					// close to an integer, e.g. 1.9999
			int tmp = QFTOLRND(tx)-1;			// e.g. -1.999 -->
			xs2 = roi_ref.xStart+tmp;
			if (xs2<0) xs2 = 0;
			xs1=xs2-tmp;
			xe1=roi_ref.xEnd;					//
			if (xe1>=img_width)
				xe1=img_width-1;
			dx=0;
			ddx=1;
		} else {
			int tmp = QFTOLRND(tx)-1;
			xs2 = roi_ref.xStart+tmp;
			if (xs2<0) xs2 = 0;
			xs1=xs2-tmp;
			xe1=roi_ref.xEnd;					//
			if (xe1>=img_width)
				xe1=img_width-1;
			dx=tx-tmp;
			ddx=1-dx;
		}
	} else {

		dx=tx-QFTOL(tx);

		if (dx<1e-4) {
			int tmp = QFTOL(tx);
			xe2 = roi_ref.xEnd + tmp;
			if (xe2>=img_width) xe2 = img_width-1;
			xe1=xe2-tmp;
			xs1=roi_ref.xStart;
			xs2=xs1+tmp;
			dx=0;
			ddx=1;
		} else if (1-dx<1e-4) {
			int tmp = QFTOL(tx) + 1;
			xe2 = roi_ref.xEnd + tmp;
			if (xe2>=img_width) xe2 = img_width-1;
			xe1=xe2-tmp;
			xs1=roi_ref.xStart;
			xs2=tmp+xs1;
			dx=0;
			ddx=1;
		} else {
			int tmp = QFTOL(tx) + 1;
			xe2 = roi_ref.xEnd + tmp;
			if (xe2>=img_width) xe2 = img_width-1;
			xe1=xe2-tmp;
			xs1=roi_ref.xStart;
			xs2=xs1 + tmp - 1;
			dx=tx+1-tmp;
			ddx=1-dx;
		}
	}

	if (ty<0) {						// moving up

		dy=QFTOLRND(ty)-ty;			// e.g. -1.2 --> dy = 0.2

		if (dy<1e-4) {							// close to an integer, e.g. 1.0002
			int tmp = QFTOLRND(ty);
			ys2 = roi_ref.yStart+tmp;
			if (ys2<0) ys2 = 0;					// make sure it is in image
			ys1=ys2-tmp;						//
			ye1=roi_ref.yEnd;					//
			if (ye1>=img_height)
				ye1=img_height-1;
			dy=0;
			ddy=1;
		} else if (1-dy<1e-4) {							// close to an integer, e.g. 1.9999
			int tmp = QFTOLRND(ty)-1;			// e.g. -1.999 -->
			ys2 = roi_ref.yStart+tmp;
			if (ys2<0) ys2 = 0;
			ys1=ys2-tmp;
			ye1=roi_ref.yEnd;					//
			if (ye1>=img_height)
				ye1=img_height-1;
			dy=0;
			ddy=1;
		} else {
			int tmp = QFTOLRND(ty)-1;
			ys2 = roi_ref.yStart+tmp;
			if (ys2<0) ys2 = 0;
			ys1=ys2-tmp;
			ye1=roi_ref.yEnd;					//
			if (ye1>=img_height)
				ye1=img_height-1;
			dy=ty-tmp;
			ddy=1-dy;
		}
	} else {

		dy=ty-QFTOL(ty);

		if (dy<1e-4) {
			int tmp = QFTOL(ty);
			ye2 = roi_ref.yEnd + tmp;
			if (ye2>=img_height) ye2 = img_height-1;
			ye1=ye2-tmp;
			ys1=roi_ref.yStart;
			ys2=ys1+tmp;
			dy=0;
			ddy=1;
		} else if (1-dy<1e-4) {
			int tmp = QFTOL(ty) + 1;
			ye2 = roi_ref.yEnd + tmp;
			if (ye2>=img_height) ye2 = img_height-1;
			ye1=ye2-tmp;
			ys1=roi_ref.yStart;
			ys2=tmp+ys1;
			dy=0;
			ddy=1;
		} else {
			int tmp = QFTOL(ty) + 1;
			ye2 = roi_ref.yEnd + tmp;
			if (ye2>=img_height) ye2 = img_height-1;
			ye1=ye2-tmp;
			ys1=roi_ref.yStart;
			ys2=ys1 + tmp - 1;
			dy=ty+1-tmp;
			ddy=1-dy;
		}
	}

	unsigned char *InPtr, *OutPtr;
	unsigned char *InRowPtr = In + xs2+ys2*img_width;
	unsigned char *OutRowPtr = Out + xs1+ys1*img_width;

	float ddx_ddy = ddx*ddy;
	float dx_ddy = dx*ddy;
	float ddx_dy = ddx*dy;
	float dx_dy = dx*dy;

	for(int y = ys1; y <= ye1; y++) {

		OutPtr = OutRowPtr;
		InPtr = InRowPtr;

		for(int x = xs1; x <= xe1; x++) {

			float p1, p2, p3, p4;

			p1 = InPtr[0];
			p2 = InPtr[1];
			p3 = InPtr[img_width];
			p4 = InPtr[img_width+1];

			OutPtr[0]=QFTOLRND(p1*ddx_ddy+p2*dx_ddy+p3*ddx_dy+p4*dx_dy);

			OutPtr++;
			InPtr ++;
		}
		InRowPtr += img_width;
		OutRowPtr += img_width;
	}

	return true;
}

// subtract two images	I1 - I2 = Out
// I1			:	input image 1
// img_width	:	input image width
// img_height	:	input image height
// I2			:	input image 2, same size as I1
// roi			:	ROI to perform the subtraction
// Out			:	signed image which contains the subtraction
void SubtractImage(unsigned char *I1, int img_width, int img_height,
	unsigned char *I2, ROI &roi, short *Out)
{
	long offset = img_width * roi.yStart + roi.xStart;

	unsigned char * pr1 = I1 + offset;
	unsigned char * pr2 = I2 + offset;
	short * prout = Out + offset;

	unsigned char *p1, *p2;
	short *pout;

	// standard 2D loop to go over each pixel in I1, and I2
	// calculate the subtraction
	// and save into Out
	for (int y=roi.yStart;y<=roi.yEnd;y++) {
		p1 = pr1;
		p2 = pr2;
		pout = prout;

		for (int x=roi.xStart;x<=roi.xEnd;x++) {
			if (p1[0] && p2[0])
				pout[0] = (int)(p1[0])-(int)(p2[0]);
			else
				pout[0] = 0;

			pout ++;
			p1++;	p2++;
		}
		pr1+=img_width;
		pr2+=img_width;
		prout+=img_width;
	}
}

// Constructor
// img_width			:	image width
// img_height			:	image height
// mt					:	motion type used in registration
Reg::Reg(int img_width, int img_height, MOTION_TYPE mt)
{
	// two laplacians are created
	// to support the pyramid based registration
	m_laplacian2d[0]=new LaplacianPyramid(img_width, img_height, MAX_LEVELS);
	m_laplacian2d[1]=new LaplacianPyramid(img_width, img_height, MAX_LEVELS);

	if (m_laplacian2d[0]==NULL || m_laplacian2d[1]==NULL) {
		printf("out of mem");
		exit(1);
	}

	//m_Sum = new long [(SEARCH_X*2+1)*(SEARCH_Y*2+1)];

	//if (m_Sum==NULL) {
	//	printf("out of mem");
	//	exit(1);
	//}

	m_width = img_width;
	m_height = img_height;

	m_parm.coarseLevel = MAX_LEVELS - 1;
#if NEW
	m_parm.fineLevel = 1;
#else
	m_parm.fineLevel = 2;
#endif

	// setup for the registration parameters
	//for (int i=m_parm.fineLevel;i<=m_parm.coarseLevel;i++) {
	for (int i=0;i<MAX_LEVELS;i++) {
#if NEW
		m_parm.iter[i] = i+2; //(i>=2)?i:2;
#else
		m_parm.iter[i] = 3;
#endif
		m_parm.motionType[i] = mt;
	}

	// 2D correlation search range
	m_parm.searchX = SEARCH_X;
	m_parm.searchY = SEARCH_Y;

	// matrix used for matrix calculation
	// since we use NRC convention, the index starts from 1
	// therefore for affine motion with 6 parameters,
	// we need to allocate 7x7 matrix, while only 6x6 is used
	m_A = new float*[7];
	for(int i = 1;i <= 6;i++)
		m_A[i] = new float[7];

	// similarly, for translation only motion model with 2 parameters
	// 3x3 matrix is allocated, while only 2x2 is used in matrix computation
	m_AT = new float*[3];
	for(int i = 1;i <= 2;i++)
		m_AT[i] = new float[3];

}

// destructor
Reg::~Reg()
{
	// free the laplacians
	if (m_laplacian2d[0]) delete m_laplacian2d[0];
	m_laplacian2d[0]=0;
	if (m_laplacian2d[1]) delete m_laplacian2d[1];
	m_laplacian2d[1]=0;
}

// set the fine and coarse level for pyramid processing
// for example, set fineLevel to be 0, and coarseLevel to be 5,
// so that the pyramid processing will go from the level 5 all
// the way to level 0, the highest resolution
// fineLevel		:	fine level the pyramid processing will go
// coarseLevel		:	coarse level the pyramid processing will go
void Reg::SetLevels(int fineLevel, int coarseLevel)
{
	// the parameters have to fall within the default maximums
	if (fineLevel>coarseLevel || coarseLevel>=MAX_LEVELS)
		return;

	m_parm.coarseLevel = coarseLevel;
	m_parm.fineLevel = fineLevel;
}

// set the motion type used in the registration
// mt				:	motion type
void Reg::SetMotionType(MOTION_TYPE mt)
{
	for (int i=0;i<MAX_LEVELS;i++) {
		m_parm.motionType[i] = mt;
	}
}

// set number of iterations in the registration
// the more iteration, the better accuracy
// and the more computational cost
// iter				:	number of iteration
void Reg::SetIterations(int iter)
{
	for (int i=0;i<MAX_LEVELS;i++) {
		m_parm.iter[i] = iter;
	}
}

// set the current inspection image as the matching template
// ins				:	current inspection image
// img_roi			:	image ROI
bool Reg::SetRef(cv::Mat I,
	ROI & img_roi)
{
	//imshow("test", I);cv::waitKey(0);
	//m_laplacian2d[0]->analysis2dMMX(ins);
	m_laplacian2d[0]->analysis2d(I);

#if 0	// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	m_laplacian2d[0]->save2file("test");
#endif

	if (!SetRefPyd(m_laplacian2d[0]->pyd, img_roi))
		return false;

	m_Roi = img_roi;

	// set the current transformation to be identity matrix
	m_T.affineIdentity();

	return true;	//identity;
}

// set the current laplacian pyramid as the matching template
// pyd					:	current pyramid as the matching template
// roi					:	input ROI
bool Reg::SetRefPyd(pyramid_laplacian &pyd, ROI &roi)
{
	// loop through all the pyramid levels
	for (int level=m_parm.coarseLevel; level >= m_parm.fineLevel; level --) {

		// update ROI at each level
		ROI roi_level = roi;

		// scale the ROI properly for each level
		for (int i=0;i<level;i++)
			roi_level *= 0.5;

		// at each pyramid level, set the matching template
		// and compute the corresponding matching matrices
		if (!SetRefLevel(pyd.levels[level],roi_level, level, m_parm.motionType[level]))
			return false;
	}

	return true;
}

// set the matching template at each pyramid level
// ll					:	pyramid level for matching template
// roi					:	inspection ROI
// level				:	current level index
// type					:	motion type
bool Reg::SetRefLevel(pyramid_laplacian_level &ll, ROI &roi, int level, MOTION_TYPE type)
{
	float gx, gy, gg, xg, yg, x, y;

	// accumulative items
	// refer to Bergen's original paper
	float Gxx = 0.0, xGxx = 0.0, yGxx = 0.0;
	float Gxy = 0.0, xGxy = 0.0, yGxy = 0.0;
	float Gyy = 0.0, xGyy = 0.0, yGyy = 0.0;

	float xyGxx = 0.0, xxGxx = 0.0, yyGxx = 0.0;
	float xyGxy = 0.0, xxGxy = 0.0, yyGxy = 0.0;
	float xyGyy = 0.0, xxGyy = 0.0, yyGyy = 0.0;

	int i, j, l, r, t, b;
	int xc, yc;

	short *ixp, *iyp;

	l = roi.xStart;
	r = roi.xEnd;
	b = roi.yStart;
	t = roi.yEnd;

	int img_width = ll.xsize;
	int img_height = ll.ysize;

	short *Ix = ll.dx;
	short *Iy = ll.dy;

	roi.center(xc, yc);

	long offset = img_width * b + l;

	short *IxRow = Ix + offset;
	short *IyRow = Iy + offset;

	for (j = b; j <= t; j++) {

		// all the accumulative items are w.r.t. the center of the region
		y = (float) (j-yc);

		ixp = IxRow;
		iyp = IyRow;

		for (i = l; i <= r; i++) {

			x = (float) (i-xc);

			// convert gx and gy to float
			gx = (float) (ixp[0]);	ixp ++;
			gy = (float) (iyp[0]);	iyp ++;

			// accumulate the sums for all accumulative items
			// again refer to Bergen's original paper
			gg = gx*gx;
			Gxx += gg;
			xg = x*gg;
			xGxx += xg;
			yg = y*gg;
			yGxx += yg;
			xxGxx += x*xg;
			xyGxx += y*xg;
			yyGxx += y*yg;
			gg = gx*gy;
			Gxy += gg;
			xg = x*gg;
			xGxy += xg;
			yg = y*gg;
			yGxy += yg;
			xxGxy += x*xg;
			xyGxy += y*xg;
			yyGxy += y*yg;
			gg = gy*gy;
			Gyy += gg;
			xg = x*gg;
			xGyy += xg;
			yg = y*gg;
			yGyy += yg;
			xxGyy += x*xg;
			xyGyy += y*xg;
			yyGyy += y*yg;

		}

		IxRow += img_width;
		IyRow += img_width;

	}

	// for different motion types, the motion parameters are estimated separately
	switch (type) {

	case MOTION_TRANSLATION:

		// compute the motion matrix for translation
		m_WT[level][0][0] = Gxx;
		m_WT[level][0][1] = m_WT[level][1][0] = Gxy;
		m_WT[level][1][1] = Gyy;

		for(i = 0;i < 2;i++)
			for(j = 0;j < 2; j++)
				m_AT[i+1][j+1] = (float)(m_WT[level][i][j]);

		if (!gaussj(m_AT, 2))
			return false;

		for(i = 0;i < 2;i++)
			for(j = 0;j < 2; j++) {
				m_WTinv[level][i][j] = m_AT[i+1][j+1];
			}

			break;

	case MOTION_SCALE_ROTATION:			// motion without x,y skew

		// compute the motion matrix for scale rotation 4x4 matrix
		m_W[level][0][0] = Gxx;
		m_W[level][0][1] = m_W[level][1][0] = Gxy;
		m_W[level][0][2] = m_W[level][2][0] = xGxx+yGxy;
		m_W[level][0][3] = m_W[level][3][0] = -yGxx+xGxy;
		m_W[level][1][1] = Gyy;
		m_W[level][1][2] = m_W[level][2][1] = xGxy+yGyy;
		m_W[level][1][3] = m_W[level][3][1] = -yGxy+xGyy;
		m_W[level][2][2] = xxGxx+2*xyGxy+yyGyy;
		m_W[level][2][3] = m_W[level][3][2] = -xyGxx-yyGxy+xxGxy+xyGyy;
		m_W[level][3][3] = yyGxx-2*xyGxy+xxGyy;

		for(i = 0;i < 4;i++)
			for(j = 0;j < 4; j++)
				m_A[i+1][j+1] = (float)(m_W[level][i][j]);

		if (!gaussj(m_A, 4))
			return false;

		// save the inverse of the motion matrix
		// since only the inverse matrix is needed in following computation
		for(i = 0;i < 4;i++)
			for(j = 0;j < 4; j++) {
				m_Winv[level][i][j] = m_A[i+1][j+1];
			}

			break;


	case MOTION_AFFINE:

		// compute the motion matrix for affine model, 6x6 matrix
		m_W[level][0][0] = Gxx;
		m_W[level][0][1] = m_W[level][1][0] = xGxx;
		m_W[level][0][2] = m_W[level][2][0] = yGxx;
		m_W[level][0][3] = m_W[level][3][0] = Gxy;
		m_W[level][0][4] = m_W[level][4][0] = xGxy;
		m_W[level][0][5] = m_W[level][5][0] = yGxy;
		m_W[level][1][1] = xxGxx;
		m_W[level][1][2] = m_W[level][2][1] = xyGxx;
		m_W[level][1][3] = m_W[level][3][1] = xGxy;
		m_W[level][1][4] = m_W[level][4][1] = xxGxy;
		m_W[level][1][5] = m_W[level][5][1] = xyGxy;
		m_W[level][2][2] = yyGxx;
		m_W[level][2][3] = m_W[level][3][2] = yGxy;
		m_W[level][2][4] = m_W[level][4][2] = xyGxy;
		m_W[level][2][5] = m_W[level][5][2] = yyGxy;
		m_W[level][3][3] = Gyy;
		m_W[level][3][4] = m_W[level][4][3] = xGyy;
		m_W[level][3][5] = m_W[level][5][3] = yGyy;
		m_W[level][4][4] = xxGyy;
		m_W[level][4][5] = m_W[level][5][4] = xyGyy;
		m_W[level][5][5] = yyGyy;

		for(i = 0;i < 6;i++)
			for(j = 0;j < 6; j++)
				m_A[i+1][j+1] = (float)(m_W[level][i][j]);

		if (!gaussj(m_A, 6))
			return false;

		// save the inverse of the motion matrix
		// since only the inverse is needed in following computation
		for(i = 0;i < 6;i++)
			for(j = 0;j < 6; j++) {
				m_Winv[level][i][j] = m_A[i+1][j+1];
			}

			break;

	default:
		printf("wrong motion type");
		return false;
	}

	return true;
}

// registration process
// so far, this function always return true
// ins					:	input inspection image
// ref2ins				:	the output motion
bool Reg::Register(cv::Mat I,
	Affine &ref2ins)
{

	//m_laplacian2d[1]->analysis2dMMX(ins);
	//unsigned char *ins = I.data;
	m_laplacian2d[1]->analysis2d(I);

	ROI roi_coarse = m_Roi;
	Affine currentT;
	currentT = m_T;

	// compute the coarse ROI and transformation
	// since the registration process starts with the coarse level
	for (int i=0;i<m_parm.coarseLevel;i++) {
		roi_coarse *= 0.5;
		currentT *= 0.5;
	}

	// global 2D correlation to find the initial translation motion
	/*
	CorrelationSearch(m_laplacian2d[1]->pyd.levels[m_parm.coarseLevel],
	m_laplacian2d[0]->pyd.levels[m_parm.coarseLevel],
	roi_coarse,
	m_parm.searchX, m_parm.searchY,
	currentT);
	*/

	/* Perform the alignment */
	for (int level = m_parm.coarseLevel; level >= m_parm.fineLevel; level--) {

		// find the current ROI and initial motion
		roi_coarse = m_Roi;
		for (int i=0;i<level;i++)
			roi_coarse *= 0.5;

		if (level != m_parm.coarseLevel) {
			currentT *= 2;
		}

		// loop through the specified iteration to estimate the motion at current level
		for (int iter = 0; iter < m_parm.iter[level]; iter++) {

			RegisterAffine(m_laplacian2d[1]->pyd.levels[level],
				m_laplacian2d[0]->pyd.levels[level],
				roi_coarse,
				level,
				m_parm.motionType[level],
				currentT);

		}
	}

	/* Rescale the motion for the remaining pyramid levels */
	for (int level = (m_parm.fineLevel - 1); level >= 0; level--) {
		currentT *= 2;
	}

	m_T = currentT;
	ref2ins = m_T;

	return true;
}

// motion : ref -> ins
bool Reg::RegisterAffine(pyramid_laplacian_level &ins, pyramid_laplacian_level &ref, ROI &roi, int level, MOTION_TYPE type, Affine &motion)
{
	if (type==MOTION_TRANSLATION)
		WarpImageTranslation(ins.data[0], ins.xsize, ins.ysize, motion, roi, ins.data[1]);
	else if (type==MOTION_AFFINE || type==MOTION_SCALE_ROTATION)
		WarpImageAffine(ins.data[0], ins.xsize, ins.ysize, motion, roi, ins.data[1]);
	else
		return false;

	SubtractImage(ins.data[1], ins.xsize, ins.ysize, ref.data[0], roi, ref.dt);

	Affine dA;
	//Reg::DeltaMotion(ref.dx, ref.dy, ref.dt, ref.xsize, ref.ysize, roi, level, type, m_Winv, m_WTinv, dA);
	DeltaMotion(ref.dx, ref.dy, ref.dt, ref.xsize, ref.ysize, roi, level, type, dA);
	/* Add the new motion to the existing motion */
	affineMultiply(motion, dA, motion);

	return true;
}

#if 1
// ivalid pixels in It are set to zero
// so those pixels should not change the outcome of this function
// therefore there is no explicit condition check
// Ix			:	x-gradient image
// Iy			:	y-gradient image
// It			:	t-gradient image
// img_width	:	image width
// img_height	:	image height
// roi			:	ROI for estimation
// level		:	pyramid level
// type			:	motion type
// dA			:	delta affine motion, the output
void Reg::DeltaMotion(short *Ix, short *Iy, short *It, int img_width, int img_height, ROI &roi, int level, MOTION_TYPE type, Affine &dA)
{
	float gx, gy, gt, f, xg, yg, x, y;
	float Gxt = 0.0, xGxt = 0.0, yGxt = 0.0, Gyt = 0.0, xGyt = 0.0, yGyt = 0.0;
	float s[7];
	float a[6];

	int i, j, l, r, t, b;
	int xc, yc;

	short *ixp, *iyp, *itp;

	/* Set up useful variables for bounding.
	* This also suppresses the need to have these evaluated at
	* the start of each for() loop.
	*/
	l = roi.xStart;
	r = roi.xEnd;
	b = roi.yStart;
	t = roi.yEnd;

	roi.center(xc, yc);

	long offset = img_width * b + l;

	short *IxRow = Ix + offset;
	short *IyRow = Iy + offset;
	short *ItRow = It + offset;

	for (j = b; j <= t; j++) {

		y = (float) (j-yc);

		ixp = IxRow;
		iyp = IyRow;
		itp = ItRow;

		for (i = l; i <= r; i++) {

			x = (float) (i-xc);

			/* convert images to float and increment pointers */
			gx = (float) (ixp[0]);	ixp ++;
			gy = (float) (iyp[0]);	iyp ++;
			gt = (float) *itp++;


			/* accumulate sums for coefficient matrix */

			Gxt += f = gx*gt;  xGxt += x*f;       yGxt += y*f;
			Gyt += f = gy*gt;  xGyt += x*f;       yGyt += y*f;

		} /* for i */

		IxRow += img_width;
		IyRow += img_width;
		ItRow += img_width;

	} /* for j */

	switch (type) {

	case MOTION_TRANSLATION:

		s[1] = Gxt;
		s[2] = Gyt;

		for(i = 0;i < 2;i++) {
			a[i] = 0.0;
			for(j = 0;j < 2;j++)
				a[i] += (float)(m_WTinv[level][i][j] * s[j+1]);
		}

		dA.affineIdentity();

		dA.A[0][2] = -a[0];
		dA.A[1][2] = -a[1];

		dA.setAffineMatrix();

		break;

	case MOTION_AFFINE:

		s[1] = Gxt;  s[2] = xGxt;  s[3] = yGxt;
		s[4] = Gyt;  s[5] = xGyt;  s[6] = yGyt;

		for(i = 0;i < 6;i++) {
			a[i] = 0.0;
			for(j = 0;j < 6;j++)
				a[i] += (float)(m_Winv[level][i][j] * s[j+1]);
		}

		dA.A[0][0] = 1.0f - a[1];
		dA.A[1][1] = 1.0f - a[5];
		dA.A[0][1] = -a[2];
		dA.A[0][2] = -a[0];
		dA.A[1][0] = -a[4];
		dA.A[1][2] = -a[3];

		dA.setAffineMatrix();

		break;

	case MOTION_SCALE_ROTATION:

		s[1] = Gxt;  s[3] = xGxt+yGyt;
		s[2] = Gyt;  s[4] = -yGxt+xGyt;

		for(i = 0;i < 4;i++) {
			a[i] = 0.0;
			for(j = 0;j < 4;j++)
				a[i] += (float)(m_Winv[level][i][j] * s[j+1]);
		}

		dA.A[0][0] = 1.0f - a[2];
		dA.A[1][1] = 1.0f - a[2];
		dA.A[0][1] = a[3];
		dA.A[0][2] = -a[0];
		dA.A[1][0] = -a[3];
		dA.A[1][2] = -a[1];

		dA.setAffineMatrix();

		break;

	default:

		return;

	} /* switch() */

	dA.setAffineOrigin(xc, yc);
	dA.changeAffineOrigin(0,0);
}

#endif

// correlation based search for translation estimation
// Ins				:	inspection image
// Ref				:	reference image
// roi				:	estimation ROI
// Nx				:	search range along x direction
// Ny				:	search range along y direction
// ref2ins			:	returned motion estimation
void Reg::CorrelationSearch(pyramid_laplacian_level &Ins,
	pyramid_laplacian_level &Ref,
	ROI &roi,
	int Nx, int Ny, Affine & ref2ins)
{
	if (!ref2ins.isIdentity())
		WarpImageTranslation(Ins.data[0], Ins.xsize, Ins.ysize, ref2ins, roi, Ins.data[1]);

	int img_width = Ins.xsize;
	int img_height = Ins.ysize;

	unsigned char *insP;
	unsigned char *refP;

	long max_sum = -1000000000;
	int xMax, yMax;
	xMax = 0;
	yMax = 0;

	for (int ys = -Ny; ys <= Ny; ys++) {
		for (int xs = -Nx; xs <= Nx; xs++) {

			//m_Sum[yy][xx]=0;
			long sum = 0;

			ROI roi_ins = roi.translate(xs, ys, img_width, img_height);
			ROI roi_ref = roi_ins.translate(-xs, -ys, img_width, img_height);

			if (roi_ref.xSize <= 0 || roi_ref.ySize <= 0)
				continue;

			unsigned char *insR = Ins.data[1] + img_width * roi_ins.yStart + roi_ins.xStart;
			unsigned char *refR = Ins.data[0] + img_width * roi_ref.yStart + roi_ref.xStart;

			unsigned char *ins;
			unsigned char *ref;

			/* compute the cross-correlations and integrate */
			for (int y = roi_ins.yStart; y <= roi_ins.yEnd; y++) {
				ins = insR;
				ref = refR;
				for (int x = roi_ins.xStart; x <= roi_ins.xEnd; x++) {
					sum += abs(ins[0] - ref[0]);
					ins ++;
					ref ++;
				}
				insR += img_width;
				refR += img_width;
			}

			sum = sum / (roi_ins.xSize * roi_ins.ySize);

			if (sum > max_sum) {
				max_sum = sum;
				xMax = xs;
				yMax = ys;
			}

		} /* for xs */
	} /* for ys */

	ref2ins.A[0][2] += xMax;
	ref2ins.A[1][2] += yMax;

	return;
}
