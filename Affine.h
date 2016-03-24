
#ifndef AFFINE_H
#define AFFINE_H

#include "ROI.h"
#include "math.h"

//
//  Affine: Affine 3x2 image transformations.
//  An affine transformation has two representations:
//     - a 2x3 matrix. The inverse of the matrix is also
//     store and is computed only when requested.
//     - a set of explicit transformation parameters: translation x,y,
//     rotation, scale x,y, and skew. The parameters are computed
//     only when requested.
//
//  The affine transformation is expressed with respect to an "origin" (xo,yo),
//  i.e.:
//      TX = A(X-Xo) + Xo
//  where Xo is the center and A is the 3x2 matrix. If Xo = 0, then the
//  transformation is applied as usual wrt the reference coordinate system.
//  This is used for expressing the transformation of a template irrespective
//  of its position in the image.
//

typedef float AffineMatrix[2][3];

class Affine
{
public:


  // constructors
  // default constructor
  Affine();

  // constructor from an existing affine
  Affine(AffineMatrix m);

  // constructor from the set of parameters
  // angle	: rotation angle
  // tx		: translation x direction
  // ty		: translation y direction
  // sx		: scale x direction
  // sy		: scale y direction
  // sk		: skew factor
  Affine(float angle, float tx, float ty, float sx, float sy, float sk);

  // destructor
  ~Affine();

  // set the affine to be the identity matrix
  void  affineIdentity();

  // set the affine to A
  void  setAffineMatrix(AffineMatrix A);

  // set the affine matrix according to the current affine parameters
  void  setAffineMatrix();

  // set the rotation angle
  void  setAffineRotation(float angle);

  // set the translation (tx, ty)
  void  setAffineTranslation(float tx, float ty);

  // set the scale factor (sx, sy)
  void  setAffineScale(float sx, float sy);

  // set the skew factor
  void  setAffineSkew(float skew);

  // set the affine origin to be (x,y)
  void  setAffineOrigin(float x, float y);

  // change the current affine origin to (x,y)
  void  changeAffineOrigin(float x, float y);

  // compute the inverse affine to out
  void  affineInvert(Affine & out);

  // apply the current affine to (x,y) to map to (ax,ay)
  void  apply(const int x, const int y, int & ax, int & ay) const;

  // apply the current affine to (x,y) to map to a floating point position (ax,ay)
  void  apply(const int x, const int y, float & ax, float & ay) const;

  // apply the inverse affine to (x,y) to map to a pixel location (ax,ay)
  void  applyInverse(const int x, const int y, int & ax, int & ay) const;

  // apply the inverse affine to (x,y) to map to a floating point location (ax,ay)
  void  applyInverse(const int x, const int y, float & ax, float & ay) const;

  // print the affine parameters for debug purpose
  void  affinePrint();

  // retrieve the rotation parameter
  float  getRotation();

  // retrieve the x translation
  float  getXTrans();

  // retrieve the y translation
  float  getYTrans();

  // retrieve the x scale factor
  float  getXScale();

  // retrieve the y scale factor
  float  getYScale();

  // retrieve the skew factor
  float  getSkew();

  // retrieve the origin
  void   getOrigin(float & x, float & y);

  // over-loaded operators
  // value passing
  Affine& operator=(Affine & T)  ;

  // check equality
  bool operator==(Affine &T);

  // affine multiplication
  Affine& operator*=(const float s);

  // reduce current affine by a factor
  Affine& ReduceByFactor(const float s);

  // translate current affine by (x,y), floating point version
  void affineTranslate(float x, float y);

  // translate current affine by (x,y), integer version
  void affineTranslate(int x, int y);

  // scale the current affine by factor (xscale, yscale)
  void affineScale(float xscale, float yscale);

  // check if current affine is identity
  int  isIdentity();

  // transform a ROI w
  ROI transformROI(ROI & w, int x, int y);

  // affine origin
  float  xo, yo;

  // affine matrix
  AffineMatrix  A;

  // affine inverse matrix
  AffineMatrix  Ainverse;

  // indicator if modified
  bool   modified;

private:

  // affine parameters
  float  rotation, xTrans, yTrans, xScale, yScale, skew;

  // apply a generic affine A to a pixel location (x,y) to get the new location (ax,ay), integer version
  void  applyGeneric(const AffineMatrix A, const int x, const int y, int & ax, int & ay) const;

  // apply a generic affine A to a pixel location (x,y) to get the new location (ax,ay), floating point version
  void  applyGeneric(const AffineMatrix A, const int x, const int y, float & ax, float & ay) const;

  // calculate the parameters from the current transformation matri
  void  computeParamFromMatrix();

  // compute the inverse affine matrix
  void  setInverse();

  // compute the affine matrix from the current parameters
  void  computeMatrixFromParam();

  // update a ROI according to the given pixel locations
  void  updateROI(int xp, int yp, int & xs, int & ys, int & xe, int & ye);

};

void  affineMultiply(Affine & Tout, Affine & T1, Affine & T2);

// fast floor function
static __inline int QFTOL(double x)
{
   x += 68719476736.0*1.5;
   return ((int*) &x)[0] >> 16;
}

static __inline int QFTOLRND(float x)
{
	int tmp;
	tmp = floor(x);
	return tmp;
}


#endif
