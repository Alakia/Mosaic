
#include "../include/ROI.h"

// constructor
// location & size
ROI::ROI(int xstart, int ystart, int xsize, int ysize):
  xSize(xsize),
  ySize(ysize),
  xStart(xstart),
  yStart(ystart),
  xEnd(xstart + xsize - 1),
  yEnd(ystart + ysize - 1),
  size(xsize*ysize)
{
}

// default constructor
ROI::ROI()
{
  xSize = ySize = xStart = yStart = xEnd = yEnd = size = 0;
}

// destructor
ROI::~ROI()
{
}

// set ROI to the specified location and size
void ROI::setROI(int xstart, int ystart, int xsize, int ysize)
{
  xSize = xsize;
  ySize = ysize;
  xStart = xstart;
  yStart = ystart;
  xEnd = xstart + xsize - 1;
  yEnd = ystart + ysize- 1;
  size = xsize*ysize;
}

// change the x and y scale of the ROI
void ROI::changeScale(float xscale, float yscale)
{
	xSize = (int)(xSize*xscale);
	ySize = (int)(ySize*yscale);
	xStart = (int)(xStart*xscale);
	yStart = (int)(yStart*yscale);
	xEnd=xStart+xSize-1;
	yEnd=yStart+ySize-1;
	size=xSize*ySize;
}

// return the center of the ROI
void ROI::center(int & x, int & y)
{
  x = xStart + xSize/2;
  y = yStart + ySize/2;
}

// return the origin of the ROI
void ROI::origin(int & x, int & y)
{
  x = xStart;
  y = yStart;
}

// scale the ROI
ROI& ROI::operator*=(float s)
{

  xStart *= s;
  yStart *= s;
  xEnd *= s;
  yEnd *= s;

  xSize = xEnd-xStart+1;
  ySize = yEnd-yStart+1;
  size = xSize*ySize;

  return(*this);

}

// translate the ROI
ROI ROI::translate(int tx, int ty, int xs, int ys)
{
  int xstart = xStart + tx;

  if(xstart < 0) xstart = 0;
  if(xstart >= xs) xstart = xs-1;

  int ystart = yStart + ty;

  if(ystart < 0) ystart = 0;
  if(ystart >= ys) ystart = ys-1;

  int xsize = (xstart + xSize > xs) ? (xs - xstart) : xSize;
  int ysize = (ystart + ySize > ys) ? (ys - ystart) : ySize;

  return ROI(xstart, ystart, xsize, ysize);

}

// crop the ROI w.r.t. the given size
ROI ROI::crop(int xsize, int ysize)
{
  ROI w(0,0,xsize,ysize);

  return crop(w);
}

// crop the ROI w.r.t. the give ROI
ROI ROI::crop(ROI &cropROI)
{
  int xs = xStart;

  if(xs < cropROI.xStart)
    xs = cropROI.xStart;

  int xe = xEnd;

  if(xe > cropROI.xEnd)
    xe = cropROI.xEnd;

  int ys = yStart;

  if(ys < cropROI.yStart)
    ys = cropROI.yStart;

  int ye = yEnd;

  if(ye > cropROI.yEnd)
    ye = cropROI.yEnd;

  return ROI(xs, ys, xe - xs + 1, ye - ys + 1);
}

// check if the ROI is empty
bool ROI::isEmpty()
{
  return xSize <= 0 || ySize <= 0;
}
