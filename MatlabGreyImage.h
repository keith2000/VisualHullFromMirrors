
#ifndef MATLABGREYIMAGE_H
#define MATLABGREYIMAGE_H

#include "mex.h"

struct mxArray_tag;

class MatlabGreyImage
{
	
public:
	
	MatlabGreyImage( const mxArray_tag* matlabMatrix );
	MatlabGreyImage();

	void Set( const mxArray_tag* matlabMatrix );
	double GetIntensity( const long  u, const long v);
	double GetIntensity( const double  u, const double v );
	long GetWidth() {return width;}
	long GetHeight() {return height;}
	
private:
	long height;
	long width;
	double * data;
		
};


#endif