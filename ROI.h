
#ifndef ROI_H
#define ROI_H

// ROI : region of interest

class ROI
{

public:

  // location and size of the ROI
  int xSize, ySize;
  int xStart, yStart;
  int xEnd, yEnd;
  int size;

  // constructor
  // xstart		:	x start point
  // ystart		:	y start point
  // xsize		:	x size
  // ysize		:	y size
  ROI(int xstart, int ystart, int xsize, int ysize);

  // default constructor
  ROI ();

  // destructor
  ~ROI();

  // scale the ROI by a factor s
  ROI& operator *= (float s);

  // set the ROI to new set of parameters
  void setROI(int xstart, int ystart, int xsize, int ysize);

  // retrieve the center of the ROI
  void center(int & x, int & y);

  // retrieve the origin of the ROI
  void origin(int & x, int & y);

  // change image resolution, mapping ROI across pyramid levels
  void changeScale(float xscale, float yscale);

  // translate the ROI by (tx, ty), limited by (xs, ys)
  ROI translate(int tx, int ty, int xs, int ys);

  // crop the ROI by the given size (xsize, ysize)
  ROI crop(int xsize, int ysize);

  // crop the ROI by the given ROI w
  ROI crop(ROI & w);

  // check if the ROI is empty, meaning the size is non-positive
  bool   isEmpty();

};
#endif
