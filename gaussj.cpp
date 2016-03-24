
#include <math.h>
#include <stdio.h>
#include "gaussj.h"

#define SWAP(a,b) {temp=(a);(a)=(b);(b)=temp;}

#define MAXSIZE 100

// gaussian iteration to solve linear systems
bool gaussj(float **a, int n)
{
    /*	int *indxc,*indxr,*ipiv;*/
    int indxc[MAXSIZE], indxr[MAXSIZE], ipiv[MAXSIZE];
	int i,icol,irow,j,k,l,ll;
	float big,dum,pivinv,temp;

	if(n > MAXSIZE) {
	  printf("m too big (%d) in gaussj \n");
	  return false;
	}
	for (j=1;j<=n;j++) ipiv[j]=0;
	for (i=1;i<=n;i++) {
		big=0.0;
		for (j=1;j<=n;j++)
			if (ipiv[j] != 1)
				for (k=1;k<=n;k++) {
					if (ipiv[k] == 0) {
						if (fabs(a[j][k]) >= big) {
							big=(float)fabs(a[j][k]);
							irow=j;
							icol=k;
						}
					} else if (ipiv[k] > 1) return false;
				}
		++(ipiv[icol]);
		if (irow != icol) {
			for (l=1;l<=n;l++) SWAP(a[irow][l],a[icol][l])
		}
		indxr[i]=irow;
		indxc[i]=icol;
		if (a[icol][icol] == 0.0)
		  return false;
		pivinv=1.0f/a[icol][icol];
		a[icol][icol]=1.0;
		for (l=1;l<=n;l++) a[icol][l] *= pivinv;
		for (ll=1;ll<=n;ll++)
			if (ll != icol) {
				dum=a[ll][icol];
				a[ll][icol]=0.0;
				for (l=1;l<=n;l++) a[ll][l] -= a[icol][l]*dum;
			}
	}
	for (l=n;l>=1;l--) {
		if (indxr[l] != indxc[l])
			for (k=1;k<=n;k++)
				SWAP(a[k][indxr[l]],a[k][indxc[l]]);
	}
	return true;
}
#undef SWAP

