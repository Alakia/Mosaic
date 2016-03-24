/*
 * Affine.h
 *
 *  Created on: Jan 1, 2011
 *      Author: root
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "Affine.h"


// default constructor
// set all the affine parameters to 0
// set the affine matrix to identity
Affine::Affine():
  rotation(0),
  xTrans(0),
  yTrans(0),
  xScale(1),
  yScale(1),
  skew(0),
  xo(0),
  yo(0)
{
  A[0][0] = A[1][1] = 1.0;
  A[0][1] = A[0][2] = 0.0;
  A[1][0] = A[1][2] = 0.0;

  setInverse();

  modified = true;
}

// constructor from the input affine matrix
// copy the affine matrix
Affine::Affine(AffineMatrix m)
{
  A[0][0] = m[0][0]; A[1][1] = m[1][1];
  A[0][1] = m[0][1]; A[0][2] = m[0][2];
  A[1][1] = m[1][1]; A[1][2] = m[1][2];

  setInverse();

  modified = true;
}

// constructor from the input affine parameters
// set all the affine parameters according to the input parameters
Affine::Affine(float angle, float tx, float ty, float sx, float sy, float sk):
  rotation(angle),
  xTrans(tx),
  yTrans(ty),
  xScale(sx),
  yScale(sy),
  skew(sk)
{
  // compute the affine matrix according to the parameters
  computeMatrixFromParam();
  setInverse();

  modified = true;
}

// destructor
// do nothing, since there no memory to free
Affine::~Affine()
{

}

// print the affine parameters for debug purpose
void Affine::affinePrint()
{
	printf("\naffine: %f %f %f\n%f %f %f\n%f %f\n", A[0][0], A[0][1], A[0][2], A[1][0], A[1][1], A[1][2], xo, yo);
}

// set the affine matrix to be the identity matrix
void  Affine::affineIdentity()
{
  A[0][0] = A[1][1] = 1.0;
  A[0][1] = A[0][2] = 0.0;
  A[1][0] = A[1][2] = 0.0;

  setInverse();

  modified = true;
}

// compute the affine parameters from the affine matrix
void  Affine::setAffineMatrix()
{
  computeParamFromMatrix();
  setInverse();

  modified = false;
}

// set the affine matrix according to the input affine matrix
// then compute the affine parameters accordingly
void  Affine::setAffineMatrix(AffineMatrix m)
{
  int  i, j;

  for(i = 0;i < 2;i++)
    for(j = 0;j < 3;j++)
      A[i][j] = m[i][j];

  computeParamFromMatrix();
  setInverse();

  modified = false;
}

// set the affine rotation parameter
// update the affine matrix
void  Affine::setAffineRotation(float angle)
{
  rotation = angle;
  computeMatrixFromParam();
  setInverse();

  modified = true;
}

// set the affine translation parameters
// update the affine matrix
void  Affine::setAffineTranslation(float tx, float ty)
{
  xTrans = tx;
  yTrans = ty;
  computeMatrixFromParam();
  setInverse();

  modified = true;
}

// set the affine scale parameters
// update the affine matrix
void  Affine::setAffineScale(float sx, float sy)
{
  xScale = sx;
  yScale = sy;
  computeMatrixFromParam();
  setInverse();

  modified = true;
}

// set the affine skew parameter
// update the affine matrix
void  Affine::setAffineSkew(float sk)
{
  skew = sk;
  computeMatrixFromParam();
  setInverse();

  modified = true;
}

// set the affine origin according to the input
void  Affine::setAffineOrigin(float x, float y)
{
  xo = x;
  yo = y;

  modified = true;
}

// change the affine origin according to the input
// update all the relevant parameters accordingly
void Affine::changeAffineOrigin(float x, float y)
{
	float dx=x-xo;
	float dy=y-yo;

	float t[2][3];

	t[0][0]=A[0][0];
	t[0][1]=A[0][1];
	t[0][2]=A[0][0]*dx+A[0][1]*dy+A[0][2];
	t[1][0]=A[1][0];
	t[1][1]=A[1][1];
	t[1][2]=A[1][0]*dx+A[1][1]*dy+A[1][2];

	A[0][0]=t[0][0];
	A[0][1]=t[0][1];
	A[0][2]=t[0][2]-dx;
	A[1][0]=t[1][0];
	A[1][1]=t[1][1];
	A[1][2]=t[1][2]-dy;

	xo=x;
	yo=y;
	setInverse();

	modified=true;
}

// compute the inverse of the current affine
// output to out
void  Affine::affineInvert(Affine & out)
{
  out.setAffineMatrix(Ainverse);
  out.setAffineOrigin(xo, yo);
}

// global utility function
// Tout = T1 * T2
void affineMultiply(Affine & Tout, Affine & T1, Affine & T2)
{
  AffineMatrix  m;

  int   i, j, k;

  for(i = 0;i < 2; i++) {

    for(j = 0;j < 3;j++) {
      m[i][j] = 0;
      for(k = 0;k < 2;k++)
	m[i][j] += T1.A[i][k] * T2.A[k][j];
    }
    m[i][2] += T1.A[i][2];
  }


  Tout.setAffineMatrix(m);

  float x, y;

  T1.getOrigin(x, y);
  Tout.setAffineOrigin(x, y);
}

// apply the current affine to point (x,y) to get (ax, ay), integer version
void Affine::apply(const int x, const int y, int & ax, int & ay) const
{
  applyGeneric(A, x, y, ax, ay);
}

// apply the current affine to point (x,y) to get (ax, ay), float version
void Affine::apply(const int x, const int y, float & ax, float & ay)  const
{
  applyGeneric(A, x, y, ax, ay);
}

// apply the inverse of current affine to (x,y) to get (ax, ay), integer version
void Affine::applyInverse(const int x, const int y, int & ax, int & ay) const
{
  applyGeneric(Ainverse, x, y, ax, ay);
}

// apply the inverse of current affine to (x,y) to get (ax, ay), float version
void Affine::applyInverse(const int x, const int y, float & ax, float & ay) const
{
  applyGeneric(Ainverse, x, y, ax, ay);
}

// generic function to apply affine matrix to point (x,y) to get (ax, ay), integer version
void  Affine::applyGeneric(const AffineMatrix A, const int x, const int y, int & ax, int & ay) const
{
  float fx, fy;

  fx = A[0][0] * x + A[0][1] * y + A[0][2] + xo - A[0][0] * xo - A[0][1] * yo;
  fy = A[1][0] * x + A[1][1] * y + A[1][2] + yo - A[1][0] * xo - A[1][1] * yo;

  ax = QFTOLRND(fx);
  ay = QFTOLRND(fy);
}

// generic function to apply affine matrix to point (x,y) to get (ax, ay), float version
void  Affine::applyGeneric(const AffineMatrix A, const int x, const int y, float & ax, float & ay) const
{
  ax = A[0][0] * x + A[0][1] * y + A[0][2] + xo - A[0][0] * xo - A[0][1] * yo;
  ay = A[1][0] * x + A[1][1] * y + A[1][2] + yo - A[1][0] * xo - A[1][1] * yo;
}

// compute the inverse matrix of the current affine matrix
void  Affine::setInverse()
{
  float d;

  // standard method to compute the inverse
  // d is the determinate
  d = A[0][0]*A[1][1] - A[0][1]*A[1][0];

  // safety check
  if (fabs(d)<0.0001) return;

  Ainverse[0][0] = A[1][1]/d;
  Ainverse[1][1] = A[0][0]/d;
  Ainverse[0][1] = -A[0][1]/d;
  Ainverse[1][0] = -A[1][0]/d;

  Ainverse[0][2] = -(Ainverse[0][0] * A[0][2] + Ainverse[0][1] * A[1][2]);
  Ainverse[1][2] = -(Ainverse[1][0] * A[0][2] + Ainverse[1][1] * A[1][2]);

}

// compute affine matrix from the parameters
void  Affine::computeMatrixFromParam()
{
  float co = (float)cos(rotation);
  float si = (float)sin(rotation);

  A[0][0] = co * xScale;
  A[1][1] = co * yScale;
  A[0][1] = -si * xScale + skew;
  A[1][0] = si * yScale;

  A[0][2] = xTrans;
  A[1][2] = yTrans;

}

// compute the affine parameters from the matrix
void  Affine::computeParamFromMatrix()
{

  if(!modified)
    return;

  yScale = (float)sqrt(A[1][0]*A[1][0] + A[1][1]*A[1][1]);
  rotation = (float)atan2(A[1][0]/yScale, A[1][1]/yScale);

  float co = (float)cos(rotation);
  float si = (float)sin(rotation);

  if(co != 0.0) {
    xScale = A[0][0]/co;
    skew = A[0][1] + si * xScale;
  } else {
    xScale = -A[0][1];
    skew = 0.0;
  }

  // this formula returns xTrans and yTrans
  // as if the origin is always at (0,0)
  xTrans = A[0][2] + xo - (A[0][0] * xo + A[0][1] * yo);
  yTrans = A[1][2] + yo - (A[1][0] * xo + A[1][1] * yo);

  modified = false;
}

// get the x scale factor
float Affine::getXScale()
{
  computeParamFromMatrix();
  return xScale;
}

// get the y scale factor
float Affine::getYScale()
{
  computeParamFromMatrix();
  return yScale;
}

// get the x translation
float Affine::getXTrans()
{
  computeParamFromMatrix();
  return xTrans;
}

// get the y translation
float Affine::getYTrans()
{
  computeParamFromMatrix();
  return yTrans;
}

// get the rotation
float Affine::getRotation()
{
  computeParamFromMatrix();
  return rotation;
}

// get the skew factor
float Affine::getSkew()
{
  computeParamFromMatrix();
  return skew;
}

// get the origin of the affine
void Affine::getOrigin(float & x, float & y)
{
  x = xo;
  y = yo;
}

// copy operator
Affine& Affine::operator=(Affine & T)
{

  setAffineMatrix(T.A);

  setAffineOrigin(T.xo, T.yo);

  return(*this);
}

// equality check operator
bool Affine::operator ==(Affine &T)
{
	return (A[0][0]==T.A[0][0] &&
			A[0][1]==T.A[0][1] &&
			A[0][2]==T.A[0][2] &&
			A[1][0]==T.A[1][0] &&
			A[1][1]==T.A[1][1] &&
			A[1][2]==T.A[1][2] &&
			xo==T.xo &&
			yo==T.yo);
}

// scaling operator
Affine& Affine::operator*=(const float s)
{
  A[0][2] *= s;
  A[1][2] *= s;

  xo *= s;
  yo *= s;

  setInverse();

  return(*this);
}

// scaling operator
Affine & Affine::ReduceByFactor(const float s)
{
  computeParamFromMatrix();

  xTrans *= s;
  yTrans *= s;
  rotation *= s;
  skew *= s;
  xScale = 1 + (xScale-1)*s;
  yScale = 1 + (yScale-1)*s;

  computeMatrixFromParam();

  setInverse();

  return(*this);
}

// translate current affine
void Affine::affineTranslate(float x, float y)
{
  A[0][2] += x;
  A[1][2] += y;

  setInverse();
}

// scale current affine
void Affine::affineScale(float x, float y)
{
  A[0][0] *= x;
  A[0][1] *= x;
  A[0][2] *= x;
  A[1][0] *= y;
  A[1][1] *= y;
  A[1][2] *= y;

  setInverse();
}

// translate current affine
void Affine::affineTranslate(int x, int y)
{
  A[0][2] += x;
  A[1][2] += y;

  setInverse();
}

// check if current affien is identity
int Affine::isIdentity()
{

  return A[0][0] == 1.0 && A[1][1] == 1.0 && A[0][1] == 0.0 && A[1][0] == 0.0
    && A[0][2] == 0.0 && A[1][2] == 0.0;

}

// update the ROI according to pixel location
void Affine::updateROI(int xp, int yp, int & xs, int & ys, int & xe, int & ye)
{
  if(xp < xs) xs = xp;
  if(xp > xe) xe = xp;
  if(yp < ys) ys = yp;
  if(yp > ye) ye = yp;
}

// transform ROI w according to size limit (x,y)
ROI Affine::transformROI(ROI & w, int x, int y)
{
  int xs = 1000;
  int xe = -1000;
  int ys = 1000;
  int ye = -1000;
  int xp, yp;

  apply(w.xStart, w.yStart, xp, yp);
  updateROI(xp, yp, xs, ys, xe, ye);
  apply(w.xStart, w.yEnd, xp, yp);
  updateROI(xp, yp, xs, ys, xe, ye);
  apply(w.xEnd, w.yEnd, xp, yp);
  updateROI(xp, yp, xs, ys, xe, ye);
  apply(w.xEnd, w.yStart, xp, yp);
  updateROI(xp, yp, xs, ys, xe, ye);

  if(xs < 0) xs = 0;
  if(ys < 0) ys = 0;

  if(xe >= x) xe = x-1;
  if(ye >= y) ye = y-1;

  return ROI(xs, ys, xe-xs+1, ye-ys+1);
}
