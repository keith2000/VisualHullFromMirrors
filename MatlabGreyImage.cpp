#include <cmath>
#include "MatlabGreyImage.h"


MatlabGreyImage::MatlabGreyImage( const mxArray_tag* matlabMatrix )
{
	
	data = mxGetPr(matlabMatrix);
	height = mxGetM(matlabMatrix);
	width = mxGetN(matlabMatrix);
}

MatlabGreyImage::MatlabGreyImage(){}


void MatlabGreyImage::Set( const mxArray_tag* matlabMatrix )
{
	
	data = mxGetPr(matlabMatrix);
	height = mxGetM(matlabMatrix);
	width = mxGetN(matlabMatrix);
}


double MatlabGreyImage::GetIntensity( const double  u, const double v)
{
	const long uInt = long( floor(u+0.5) );
	const long vInt = long( floor(v+0.5) );
 
	return GetIntensity( uInt, vInt );
}

double MatlabGreyImage::GetIntensity( const long  u, const long v)
{

	if (  u > width ) return 0;
	if (  v > height ) return 0;
	if (  u < 1 ) return 0;
	if (  v < 1 ) return 0;
	
	const long cRowInd = v-1;
	const long cColInd = u-1;

	return data[ cRowInd + cColInd * height ];
}



