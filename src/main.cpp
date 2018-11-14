#include <opencv2/opencv.hpp>

#include <stdlib.h>
#include <math.h>
#include "../include/CommonDefines.h"
#include "../include/Affine.h"
#include "../include/StablizerWrapper.h"

using namespace cv;
using namespace std;

bool stop = false;

//bool WarpImageAffine(unsigned char *In,
//					 int img_width, int img_height,
//					 Affine &mot,						// mot : from ref -> Ins
//					 ROI &roi_ref,
//					 unsigned char *Out);

int main(int argc, char **argv)
{
	VideoCapture cap;
	Mat frame(720, 1280, CV_8UC3);
	cap.open(0);

	cap.set(CV_CAP_PROP_FRAME_WIDTH,320);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT,240);
	int img_width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
	int img_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);

	int out_width = 1280, out_height = 960;
	Mat mat_frame_3u, mat_img_1u, col_frame, pattern_frame, first_frame;
	Mat Output(out_height, out_width, CV_8UC3), Outputstab(img_height,img_width,CV_8UC3);


	memset(Output.data, 0, img_width * img_height * 3);
	memset(Outputstab.data, 0, img_width * img_height * 3);

	StablizerWrapper stab(MOTION_AFFINE);//MOTION_TRANSLATION,MOTION_SCALE_ROTATION,MOTION_AFFINE
	ROI roi_stab, roi_ins, roi_stabl;

	if(!cap.isOpened())
	    {
	        cout << "could not open the camera video source" << endl;
	        return -1;
	    }

	double rate = cap.get(CV_CAP_PROP_FPS);
	int delay = 1000/rate;

	roi_stab.setROI(10,10,img_width - 10,img_height - 10);
	roi_ins.setROI(10,10,img_width - 10,img_height - 10);
	roi_stabl.setROI(10,10,img_width - 10,img_height - 10);

	int frm = 0;
	int c;
	double timecount1, timecount2;
	while(1)
	{
		if(frm == 0)
		{
			cap >> first_frame;
		}
		cap >> mat_frame_3u;
		cvtColor(first_frame, mat_img_1u, CV_BGR2GRAY);
		timecount1 = cv::getTickCount();

		//stab.Stablization(mat_img_1u, img_width, img_height, roi_stabl, mat_frame_3u, img_width, img_height, Outputstab, 4, 4);
		stab.Mosaic(mat_img_1u,  img_width,  img_height, roi_stab, mat_frame_3u, img_width, img_height, Output, out_width, out_height, roi_ins, 1, 1, 0, 4);

		//stab then mosaic
		//stab.Stablization(mat_img_1u, img_width, img_height, roi_stabl, mat_frame_3u, img_width, img_height, Outputstab, 4, 4);
		//stab.Mosaic(first_frame,  img_width,  img_height, roi_stab, mat_frame_3u, img_width, img_height, Output, out_width, out_height, roi_ins, 1, 1, 0, 4);

		timecount2 = (cv::getTickCount() - timecount1)/cv::getTickFrequency();
		printf("timecount is %f \n", timecount2);

		imshow("Capture", Output);
		c = waitKey(10)&0xFF;
		c = tolower(c);
		if(c == 27 || c == 'q')
		{
			break;
		}
		frm++;
	}
	return 1;

}



