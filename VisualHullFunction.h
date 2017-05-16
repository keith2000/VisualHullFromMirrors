

#ifndef VISUALHULLFUNCTION_H
#define VISUALHULLFUNCTION_H


#include <vector>

#include "mex.h"

#include "polygonizer.h"
#include "MatlabGreyImage.h"
#include "Matrix4x4.h"


struct mxArray_tag;

struct PerspeciveCameraInernals
{
	double efl;
	double u0;
	double v0;
};

class VisualHullFunction: public ImplicitFunction 
{
public:
	
	VisualHullFunction( const mxArray_tag* imgCell,
		const mxArray_tag* internalsCell, 
		const mxArray_tag* poseCell );


	//VisualHullFunction(){};

	 VisualHullFunction( const mxArray_tag* imgCell,
		const mxArray_tag* internalsCell, 
		const mxArray_tag* poseCell, const int numSilPerImg );
	
	double eval (double x, double y, double z);
	
private:
	std::vector<MatlabGreyImage> imgVec;
	std::vector<Matrix4x4> poseVec;
	std::vector<PerspeciveCameraInernals> internalsVec;
	int numCams;
};


#endif