// Stablizer.cpp : implementation file
//

#include "../include/Affine.h"

#include "../include/reg.h"
#include "../include/Stablizer.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>


Stablizer::Stablizer(MOTION_TYPE type)
{
	m_alpha = 0.0;

	m_Reg = 0;

	m_type = type;

}

Stablizer::~Stablizer()
{
	if (m_Reg) delete m_Reg;
}

void Stablizer::Reset()
{
	if (m_Reg) delete m_Reg;
	m_Reg = NULL;

	m_global_T.affineIdentity();
	m_current_T.affineIdentity();
	m_current_T_save.affineIdentity();
	m_last_templ_update_T.affineIdentity();

}

void Stablizer::SetMotionType(MOTION_TYPE type)
{
	m_type = type;
	Reset();
}

void Stablizer::SetIterations(int iter)
{
	if (m_Reg)
		m_Reg->SetIterations(iter);
}

bool Stablizer::LargeMotion(Affine m, int xc, int yc, int maxx, int maxy)
{
	m.changeAffineOrigin(xc, yc);

	//if (fabs(m.A[0][2])>MAX_X || fabs(m.A[1][2])>)
	if (fabs(m.A[0][2])>maxx || fabs(m.A[1][2])>maxy ||
		fabs(m.getXScale()-1)>0.1 || fabs(m.getYScale()-1)>0.1 ||
		fabs(m.getSkew())>0.1 )
		return true;
	else
		return false;
}

bool Stablizer::SmallMotion(Affine m)
{
	if (fabs(m.A[0][2])<MIN_X || fabs(m.A[1][2])<MIN_Y)
		return true;
	else
		return false;
}

// rgb4			: input RGB
// rgb3Out		: output RGB, same dimension as rgb4
void Stablizer::Stablization(cv::Mat I1, int imgwidth, int imgheight, ROI &stab_roi, cv::Mat rgb4, int rgb_width, int rgb_height, cv::Mat rgb3Out, int fineLevel, int coarseLevel)
{
	/*imshow("test", I1);
	cv::waitKey(0);
	imshow("test", rgb4);
	cv::waitKey(0);
	imshow("test", rgb3Out);
	cv::waitKey(0);*/

	static bool refSet = false;
	static int static_frame_count;

	ROI roi;
	//roi.setROI(5,5,imgwidth-10,imgwidth-10);
	//roi.setROI(MAX_X,MAX_Y,imgwidth-MAX_X*2,imgwidth-MAX_Y*2);

	if (m_Reg==0) {	// first frame
		m_Reg = new Reg(imgwidth, imgheight, m_type);
		if (m_Reg==0) {
			printf("out of mem");
			exit(1);
		}
		m_Reg->SetLevels(fineLevel,coarseLevel);

		refSet = m_Reg->SetRef(I1, stab_roi);

		//roi.setROI( (rgb_width-imgwidth)/2, (rgb_height-imgheight)/2, imgwidth, imgheight);

		// all the transformation are identity at this point
		roi.setROI(0,0,rgb_width, rgb_height);
		WarpRGBTranslation(rgb4, rgb_width, rgb_height, rgb3Out, roi, m_global_T.A[0][2], m_global_T.A[1][2]);
		return;
	}

	if (refSet) {	// if the templ is already initialized
		m_Reg->Register(I1, m_current_T);	// m_current_T : ref2ins
	}

	if (!LargeMotion(m_current_T, imgwidth/2, imgheight/2, (imgwidth-stab_roi.xSize)/2, (imgheight-stab_roi.ySize)/2))
		affineMultiply(m_global_T, m_current_T, m_last_templ_update_T);
	else {
		if (m_Reg->SetRef(I1, stab_roi))
			m_last_templ_update_T = m_global_T;
	}

	if (m_type==MOTION_AFFINE || m_type==MOTION_TRANSLATION) {	// for motion type other than scale-rotation,
		// perform motion attenuation, that is,
		// to attenuate large dominant translational
		// motion, so that the stablization can follow
		// camera motion such as pan-tilt movement
		if (LargeMotion(m_global_T, imgwidth/2, imgheight/2, (imgwidth-stab_roi.xSize)/2, (imgheight-stab_roi.ySize)/2)) {	// important
			// need to re-start from current frame

			//m_global_T.changeAffineOrigin(imgwidth/2, imgheight/2);
			m_global_T.ReduceByFactor(0.8);
			//m_global_T.changeAffineOrigin(0,0);

			if (m_Reg->SetRef(I1, stab_roi))
				m_last_templ_update_T = m_global_T;

			//	} else if (m_current_T==m_current_T_save) {	// no change in the current estimated motion

		} else if (LargeMotion(m_global_T, imgwidth/2, imgheight/2, (imgwidth-stab_roi.xSize)/4, (imgheight-stab_roi.ySize)/4)) {	// after one second, update template

			//m_global_T.changeAffineOrigin(imgwidth/2, imgheight/2);
			m_global_T.ReduceByFactor(0.9);
			//m_global_T.changeAffineOrigin(0,0);

			if (m_Reg->SetRef(I1, stab_roi))
				m_last_templ_update_T = m_global_T;
		}

	}

	if (static_frame_count>3000 || !refSet) {
		if (m_Reg->SetRef(I1, stab_roi))
			m_last_templ_update_T = m_global_T;
		static_frame_count = 0;
	} else {
		static_frame_count ++;
	}


	roi.setROI(0,0,rgb_width, rgb_height);
	if (m_type == MOTION_TRANSLATION)
		//WarpRGBTranslation(rgb4, rgb_width, rgb_height, rgb3Out, roi, m_global_T.A[0][2], m_global_T.A[1][2]);
		WarpRGBTranslation(rgb4, rgb_width, rgb_height, rgb3Out, roi, m_global_T.A[0][2], m_global_T.A[1][2]);
	else if (m_type == MOTION_AFFINE || m_type == MOTION_SCALE_ROTATION)
		//WarpRGBAffine(rgb4, rgb_width, rgb_height, rgb3Out, roi, m_global_T);
		WarpRGBAffine(rgb4, rgb_width, rgb_height, rgb3Out, roi, m_global_T);

}

// I1			: input grayscale, which is used to do the estimation
// imgwidth		: input image width
// imgheight	: input image height
// roi_stab		: specifies the effective region for estimation
// rgb4			: input RGB
// in_width		: input RGB width
// in_height	: input RGB height
// rgb3Out		: output RGB, usually a larger image to contain the mosaic
// out_width	: output RGB width
// out_height	: output RGB height
// roi_ins		: specifies the region for mosaicking in the current frame
// xscale		: scale factor
// yscale		: scale factor, used to make mosaic
// fineLevel	: fine level to estimate motion
// coarseLevel	: coarse level to estimate motion
void Stablizer::Mosaic(cv::Mat I1, int imgwidth, int imgheight, ROI &stab_roi, cv::Mat rgb4, int in_width, int in_height, cv::Mat rgb3Out, int out_width, int out_height, ROI &roi_ins, float xscale, float yscale, int fineLevel, int coarseLevel)
{
	static bool refSet = false;
	static int static_frame_count;
	//imshow("test", rgb4);cv::waitKey(0);imshow("test", rgb3Out);cv::waitKey(0);
	if (m_Reg==0) {	// first frame
		m_Reg = new Reg(imgwidth, imgheight, m_type);
		if (m_Reg==0) {
			printf("out of mem");
			exit(1);
		}
		m_Reg->SetLevels(fineLevel,coarseLevel);

		refSet = m_Reg->SetRef(I1, stab_roi);

		// m_global_T is identity at this point
		WarpRGBAffineMosaic(rgb4, in_width, in_height, rgb3Out, out_width, out_height, roi_ins, m_global_T, xscale, yscale);

		return;
	}

	if (refSet) {	// if the templ is already initialized
		m_Reg->Register(I1, m_current_T);	// m_current_T : ref2ins
	}


	// calculate the current global T
	affineMultiply(m_global_T, m_current_T, m_last_templ_update_T);

	// if the current global T is big, we'll reset the template
	if (LargeMotion(m_global_T, imgwidth/2, imgheight/2, (imgwidth-stab_roi.xSize)/4, (imgheight-stab_roi.ySize)/4)) { // large translation
		if (m_Reg->SetRef(I1, stab_roi))
			m_last_templ_update_T = m_global_T;
	}

	// if the current global T is too big, we'll re-initialize the mosaic

	//if (static_frame_count>3000 || !refSet) {	// update every 100 seconds
	if (!refSet)
		if (m_Reg->SetRef(I1, stab_roi))
			m_last_templ_update_T = m_global_T;

	bool success = WarpRGBAffineMosaic(rgb4, in_width, in_height, rgb3Out, out_width, out_height, roi_ins, m_global_T, xscale, yscale);

	// if the warping is not successful, reset the mosaic
	if (!success) {

		refSet = m_Reg->SetRef(I1, stab_roi);
		m_global_T.affineIdentity();
		m_current_T.affineIdentity();
		m_current_T_save.affineIdentity();
		m_last_templ_update_T.affineIdentity();

		memset(rgb3Out.data, 0, out_height*out_width*3);

		// m_global_T is identity at this point
		WarpRGBAffineMosaic(rgb4, in_width, in_height, rgb3Out, out_width, out_height, roi_ins, m_global_T, xscale, yscale);

	}
}


// warp the current input image onto the output image to form the mosaic
// with the estimated affine
// mot: ref2ins, orgin at (0,0)
// xscale, yscale: scale factor in constructing the mosaic
// roi_ins: the useful roi in the ins frame

// return 1 if mosaic is successful
// return 0 if mosaic is not done
bool Stablizer::WarpRGBAffineMosaic(cv::Mat rgb4, int in_width, int in_height, cv::Mat rgb3Out, int out_width, int out_height, ROI &roi_ins, Affine& mot, float xscale, float yscale)
{
	// mosaic_affine		:	ref2ins, the direction is the same as mot, or as in the stablization
	//							however, now the reference frame is enlarged and translated, so the resulting
	//							affine is different from the stablization affine
	// mosaic_affine_inv	:   the inverse of the mosaic affine, which is used to calculate the target rectangle
	//							mot (ref2ins) origin at (0,0) -> invert -> change scale -> translate to the center of output


	Affine mosaic_affine_inv;
	Affine mosaic_affine;

	mot.affineInvert(mosaic_affine_inv);							// get ins2ref
	mosaic_affine_inv.affineScale(xscale, yscale);					// scale
	mosaic_affine_inv.affineTranslate(out_width/10, out_height/10);	// translate to the center of reference
	// now mosaic_affine_inv maps ins to ref as desired

	ROI roi_ref;
	// compute the effective roi in the reference frame
	roi_ref = mosaic_affine_inv.transformROI(roi_ins, out_width, out_height);

	// if the effective mosaic is close to the boundary, we need to reset the mosaic
	if (roi_ref.xStart<10 || roi_ref.yStart<10 || roi_ref.xEnd>out_width-10 || roi_ref.yEnd>out_height-10) {
		return false;
	}

	// if the transformation is off, we need to reset the mosaic
	if (roi_ref.xSize>in_width*xscale*2 || roi_ref.ySize>in_height*yscale*2 || roi_ref.xSize<in_width*xscale/2 || roi_ref.ySize<in_height*yscale/2) {
		return false;
	}

	// get the forward mosaic affine
	mosaic_affine_inv.affineInvert(mosaic_affine);

	// pitch of the input image
	long pitch4 = in_width * 3;//4to3
	long pitch3 = out_width * 3;

	int i,j, intx, inty;
	unsigned char *outRow, *outCol, temp;
	unsigned char *Prgb3Out = rgb3Out.data, *Prgb4 = rgb4.data;

	// i,j in the reference frame
	for(j = roi_ref.yStart,outRow=Prgb3Out+j*pitch3;j<=roi_ref.yEnd;j++,outRow+=pitch3) {

		float fj = j;

		float trsx_fj = mosaic_affine.A[0][1]*fj + mosaic_affine.A[0][2];
		float trsy_fj = mosaic_affine.A[1][1]*fj + mosaic_affine.A[1][2];
		//imshow("test", rgb3Out);cv::waitKey(0);
		for(i=roi_ref.xStart,outCol=outRow+i*3;i<=roi_ref.xEnd;i++,outCol+=3) {
			float fi,yy,xx;
			unsigned char *ptemp = &(rgb3Out.at<cv::Vec3b>(j, i)[0]);
			fi = (float)i;

			if (i==j) {					// special processing
				outCol[0]=0;
				outCol[1]=0;
				outCol[2]=0;
				continue;
			}

			yy = mosaic_affine.A[1][0]*fi + trsy_fj;
			xx = mosaic_affine.A[0][0]*fi + trsx_fj;

			inty = (int) QFTOL(yy);
			intx = (int) QFTOL(xx);

			// intx,inty are coordinates in ins frame
			// need to check for minus intx, inty as well
			// the minus check is skipped by (unsigned int) type enforcer
			// since the minus value will be a large positive value (>2^15 ~= 32000) after the type change
			//if (((unsigned int) intx) < in_width-1 && ((unsigned int) inty) <in_height-1) {

			if (intx>0 && inty>0 && intx<in_width-1 && inty<in_height-1) {
				float xfrac = xx - (float)(intx);
				float yfrac = yy - (float)(inty);

				unsigned char *ptr1= Prgb4+inty*pitch4+intx*3;//4to3
				unsigned char *ptr2= ptr1 + pitch4;//

				float y1val= ((float)ptr1[0]) + xfrac*((float)ptr1[3]-(float)ptr1[0]);//4to3
				float y2val= ((float)ptr2[0]) + xfrac*((float)ptr2[3]-(float)ptr2[0]);//4to3
				float val=y1val + yfrac*(y2val-y1val);

				if (val>255) val = 255;

				outCol[0]= (unsigned char)val;
				ptemp[0] = (unsigned char)val;
				//rgb3Out[j*pitch3 + i*3] = (unsigned char)val;

				y1val= ((float)ptr1[1]) + xfrac*((float)ptr1[4]-(float)ptr1[1]);//5to4
				y2val= ((float)ptr2[1]) + xfrac*((float)ptr2[4]-(float)ptr2[1]);//5to4
				val=y1val + yfrac*(y2val-y1val);

				if (val>255) val = 255;

				outCol[1]= (unsigned char)val;
				ptemp[1] = (unsigned char)val;
				//rgb3Out[j*pitch3 + i*3 + 1] = (unsigned char)val;

				y1val= ((float)ptr1[2]) + xfrac*((float)ptr1[5]-(float)ptr1[2]);//6to5
				y2val= ((float)ptr2[2]) + xfrac*((float)ptr2[5]-(float)ptr2[2]);//6to5
				val=y1val + yfrac*(y2val-y1val);

				if (val>255) val = 255;

				outCol[2]= (unsigned char)val;
				ptemp[2] = (unsigned char)val;
				//rgb3Out[j*pitch3 + i*3 + 2] = (unsigned char)val;

			}
			//else {
			//	outCol[0]=0;
			//	outCol[1]=0;
			//	outCol[2]=0;
			//}
		}
	}

	return true;
}

void Stablizer::WarpRGBAffine(cv::Mat rgb4, int img_width, int img_height, cv::Mat rgb3Out, ROI &roi_ref, Affine& mot)
{
	roi_ref = roi_ref.crop(img_width, img_height);

	memset(rgb3Out.data, 0, img_width * img_height * 3);

	long pitch4 = img_width * 3;//4to3
	long pitch3 = img_width * 3;


	int i,j, intx, inty;
	unsigned char *outRow, *outCol;


	for(j = roi_ref.yStart,outRow=rgb3Out.data+j*pitch3;j<=roi_ref.yEnd;j++,outRow+=pitch3) {

		float fj = j;

		float trsx_fj = mot.A[0][1]*fj + mot.A[0][2];
		float trsy_fj = mot.A[1][1]*fj + mot.A[1][2];

		for(i=roi_ref.xStart,outCol=outRow+i*3;i<=roi_ref.xEnd;i++,outCol+=3) {
			float fi,yy,xx;
			unsigned char *ptemp = &(rgb3Out.at<cv::Vec3b>(j, i)[0]);
			fi = (float)i;

			yy = mot.A[1][0]*fi + trsy_fj;
			xx = mot.A[0][0]*fi + trsx_fj;

			inty = (int) QFTOL(yy);
			intx = (int) QFTOL(xx);

			// need to check for minus intx, inty as well
			// the minus check is skipped by (unsigned int) type enforcer
			// since the minus value will be a large positive value after the type change
			if (((unsigned int) intx) < img_width-1 && ((unsigned int) inty) <img_height-1) {
				float xfrac = xx - (float)(intx);
				float yfrac = yy - (float)(inty);

				unsigned char *ptr1= rgb4.data+inty*pitch4+intx*3;//4to3
				unsigned char *ptr2= ptr1 + pitch4;

				float y1val= ((float)ptr1[0]) + xfrac*((float)ptr1[3]-(float)ptr1[0]);//4to3
				float y2val= ((float)ptr2[0]) + xfrac*((float)ptr2[3]-(float)ptr2[0]);//4to3
				float val=y1val + yfrac*(y2val-y1val);

				if (val>255) val = 255;

				outCol[0]= (unsigned char)val;
				ptemp[0] = (unsigned char)val;

				y1val= ((float)ptr1[1]) + xfrac*((float)ptr1[4]-(float)ptr1[1]);//5to4
				y2val= ((float)ptr2[1]) + xfrac*((float)ptr2[4]-(float)ptr2[1]);//5to4
				val=y1val + yfrac*(y2val-y1val);

				if (val>255) val = 255;

				outCol[1]= (unsigned char)val;
				ptemp[1] = (unsigned char)val;

				y1val= ((float)ptr1[2]) + xfrac*((float)ptr1[5]-(float)ptr1[2]);//6to5
				y2val= ((float)ptr2[2]) + xfrac*((float)ptr2[5]-(float)ptr2[2]);//6to5
				val=y1val + yfrac*(y2val-y1val);

				if (val>255) val = 255;

				outCol[2]= (unsigned char)val;
				ptemp[2] = (unsigned char)val;
			}
			else {
				outCol[0]=0;
				outCol[1]=0;
				outCol[2]=0;
			}
		}
	}

}

void Stablizer::WarpRGBTranslation(cv::Mat rgb4, int img_width, int img_height, cv::Mat rgb3Out, ROI &roi_ref, float tx, float ty)
{
	/*imshow("test", rgb4);
	cv::waitKey(0);
	imshow("test", rgb3Out);
	cv::waitKey(0);*/


	float dx,dy,ddx,ddy;
	roi_ref = roi_ref.crop(img_width, img_height);


	memset(rgb3Out.data, 0, img_width * img_height * 3);

	int xs1, xs2, ys1, ys2, xe1, xe2, ye1, ye2;

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
		} else if (1-dx<1e-4) {							// close to an integer, e.g. 1.9999
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

	unsigned char *rgb4Ptr, *rgb3Ptr;
	unsigned char *rgb4RowPtr = rgb4.data + (xs2+ys2*img_width)*3;//4to3
	unsigned char *rgb3RowPtr = rgb3Out.data + (xs1+ys1*img_width)*3;

	unsigned char *rgb4temp, *rgb3temp;
	rgb4temp = &(rgb4.at<cv::Vec3b>(ys2, xs2)[0]);
	rgb3temp = &(rgb3Out.at<cv::Vec3b>(ys1, xs1)[0]);
	//unsigned char *rgb4temp1, *rgb3temp1;

	long pitch4 = img_width * 3;//4to3
	long pitch3 = img_width * 3;

	float ddx_ddy = ddx*ddy;
	float dx_ddy = dx*ddy;
	float ddx_dy = ddx*dy;
	float dx_dy = dx*dy;

	//for(int y = ys1; y <= ye1; y++) {
	// considering that the bilinear interporation need an extra line
	// it is more efficient to ignore the last row
	for(int y = ys1; y < ye1; y++) {

		//rgb3Ptr = rgb3RowPtr;
		//rgb4Ptr = rgb4RowPtr;

		rgb3Ptr = rgb3temp;
		rgb4Ptr = rgb4temp;
		//for(int x = xs1; x <= xe1; x++) {
		// similarly, the bilinear interp. needs extra column
		for(int x = xs1; x < xe1; x++) {


			float p1, p2, p3, p4;

			// R
			p1 = rgb4Ptr[0];
			p2 = rgb4Ptr[3];//4to3
			p3 = rgb4Ptr[pitch4];
			p4 = rgb4Ptr[pitch4+3];//4to3

			rgb3Ptr[0]=QFTOLRND(p1*ddx_ddy+p2*dx_ddy+p3*ddx_dy+p4*dx_dy);

			rgb3Ptr++;

			// G
			p1 = rgb4Ptr[1];
			p2 = rgb4Ptr[4];//5to4
			p3 = rgb4Ptr[pitch4+1];
			p4 = rgb4Ptr[pitch4+4];//5to4

			rgb3Ptr[0]=QFTOLRND(p1*ddx_ddy+p2*dx_ddy+p3*ddx_dy+p4*dx_dy);

			rgb3Ptr++;

			// B
			p1 = rgb4Ptr[2];
			p2 = rgb4Ptr[5];//6to5
			p3 = rgb4Ptr[pitch4+2];
			p4 = rgb4Ptr[pitch4+5];//6to5

			rgb3Ptr[0]=QFTOLRND(p1*ddx_ddy+p2*dx_ddy+p3*ddx_dy+p4*dx_dy);

			rgb3Ptr++;

			rgb4Ptr += 3;//4to3
		}


		rgb3temp += pitch3;
		rgb4temp += pitch4;
	}
}
