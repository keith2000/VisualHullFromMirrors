#include "Matrix4x4.h"
#include "mex.h"

void Matrix4x4::FormFromMatlabArray(const mxArray* M)
{
	for(int c=0;c<4;c++)
		for(int r=0;r<4;r++)
			element[r][c]=mxGetPr(M)[c*4+r];
		
}


Matrix4x4::Matrix4x4(const mxArray* M)
{
	for(int c=0;c<4;c++)
		for(int r=0;r<4;r++)
			element[r][c]=mxGetPr(M)[c*4+r];
		
}

