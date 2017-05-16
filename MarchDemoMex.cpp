//--------------------------------------------------------------
// file: MarchDemoMex.cpp - Matlab Mex function application
//--------------------------------------------------------------
#include <iostream>
#include <math.h>

extern "C" {
#include "mex.h"
}
#include "polygonizer.h"
#include "VisualHullFunction.h"
#include "MatlabGreyImage.h"



void MarchDemoMex( int nlhs, mxArray *plhs[], int nrhs, const mxArray  *prhs[] )
{
	if (nlhs!=2)
		mexErrMsgTxt("Two output arguments required: p, k");

	if (nrhs!= 7)
		mexErrMsgTxt("Seven output arguments required: image cell array, internals cell array, pose cell array, cell width, num cells across, starting point, doTet");

	
		if (!mxIsCell(prhs[0]))
			mexErrMsgTxt("image cell array must be in a cell array");
		if (!mxIsCell(prhs[1]))
			mexErrMsgTxt("internals cell array must be in a cell array");
		if (!mxIsCell(prhs[2]))
			mexErrMsgTxt("pose cell array must be in a cell array");
				
		const int numCams = mxGetNumberOfElements( prhs[0] ); //number of camera centres = number of cameras
		
		if ( numCams < 2 ) 
			mexErrMsgTxt("Need at least 2 views");

		if ( numCams != mxGetNumberOfElements( prhs[1] ) ) 
			mexErrMsgTxt("no. images != no. internals");
		if ( numCams != mxGetNumberOfElements( prhs[1] ) ) 
			mexErrMsgTxt("no. images != no. poses");

		if ( mxGetNumberOfElements( prhs[3] ) != 1 ) 
			mexErrMsgTxt("cell width must be scalar");

		if ( mxGetNumberOfElements( prhs[4] ) != 1 ) 
			mexErrMsgTxt("no. cells across must be scalar");

		if ( mxGetNumberOfElements( prhs[5] ) != 3 ) 
			mexErrMsgTxt("start point must have 3 elements: x, y, z");

		if ( mxGetNumberOfElements( prhs[6] ) != 1 ) 
			mexErrMsgTxt("doTet must be scalar: 1 or 0");



	VisualHullFunction vh( prhs[0], prhs[1], prhs[2] );




	const double cellSize = (mxGetPr(prhs[3]))[0];
	const int howFar = int((mxGetPr(prhs[4]))[0]);

	const double xStart = (mxGetPr(prhs[5]))[0];
	const double yStart = (mxGetPr(prhs[5]))[1];
	const double zStart = (mxGetPr(prhs[5]))[2];

	
	



	//mexPrintf("cellSize: %g\n", cellSize);
	//mexPrintf("howFar: %d\n", howFar);

	//mexPrintf("vh.eval(  xStart, yStart, zStart ): %g\n", vh.eval(  xStart, yStart, zStart ) );
	

	//mexErrMsgTxt("stop");
	
	Polygonizer pol( &vh, cellSize, howFar );
	
	
	bool doTet = (mxGetPr(prhs[6])) > 0;
	
	if ( doTet )
		pol.march(TET, xStart, yStart, zStart );
	else
		pol.march(NOTET, xStart, yStart, zStart );
	

	/*/
	MatlabGreyImage im( prhs[2] );
	
double uu = 7;
double vv = 89;

	mexPrintf("im.GetIntensity( %g,%g ): %g\n", uu, vv, 
		im.GetIntensity( uu, vv ) );

		mexPrintf("height: %d\n", im.GetHeight() );
		mexPrintf("width: %d\n", im.GetWidth() );

  /*/


	plhs[0]= mxCreateDoubleMatrix( 3, pol.no_vertices(), mxREAL);	
	
	for( int vertLoop = 0; vertLoop < pol.no_vertices(); vertLoop++ )
	{
		(mxGetPr(plhs[0]))[vertLoop*3 + 0] = (pol.get_vertex( vertLoop )).x;
		(mxGetPr(plhs[0]))[vertLoop*3 + 1] = (pol.get_vertex( vertLoop )).y;
		(mxGetPr(plhs[0]))[vertLoop*3 + 2] = (pol.get_vertex( vertLoop )).z;
	}
	

	plhs[1]= mxCreateDoubleMatrix( 3, pol.no_triangles(), mxREAL);	
	
	for( int triLoop = 0; triLoop < pol.no_triangles(); triLoop++ )
	{
		(mxGetPr(plhs[1]))[triLoop*3 + 0] = (pol.get_triangle( triLoop )).v0 + 1;// +1 since matlab counts array indices from 1 not 0 like C++ does
		(mxGetPr(plhs[1]))[triLoop*3 + 1] = (pol.get_triangle( triLoop )).v1 + 1;
		(mxGetPr(plhs[1]))[triLoop*3 + 2] = (pol.get_triangle( triLoop )).v2 + 1;
	}
	

} // end MarchDemoMex()

extern "C" {
  //--------------------------------------------------------------
  // mexFunction - Entry point from Matlab. From this C function,
  //   simply call the C++ application function, above.
  //--------------------------------------------------------------
  void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray  *prhs[] )
  {
    MarchDemoMex(nlhs, plhs, nrhs, prhs);
  }
}


