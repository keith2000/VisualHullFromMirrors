extern "C" {
#include "mex.h"
}

#include "Wm3Matrix3.h"
using namespace Wm3;


/*/
To compile from within Matlab:


cd C:\Keith\Work\C++\Mex\
mex -v  ...
C:\Keith\Work\C++\Mex\Rotations\MatrixToAxisAngleMex.cpp ...
C:\Keith\Library\C++\MagicSoftware\Sdk\Library\Release\MagicFM.lib ...
-IC:\Keith\Library\C++\MagicSoftware\Sdk\Include
/*/
  

extern "C" {
void MatrixToAxisAngle( int nlhs, mxArray *plhs[], int nrhs, const mxArray  *prhs[] )
{
	
	////////////////////BEGIN ARGUMENT CHECK//////////////////////////////
	if ( nrhs != 1 )
	{
		mexErrMsgTxt("1 input argument required");
	}
	
	if ( ( mxGetM(prhs[0]) != 3 ) || ( mxGetN(prhs[0]) != 3 ) )
	{
		mexErrMsgTxt("Need a 3x3 matrix as input");
	}
	////////////////////END ARGUMENT CHECK//////////////////////////////
	Matrix3<double> M(
		mxGetPr(prhs[0])[0], mxGetPr(prhs[0])[3], mxGetPr(prhs[0])[6],
		mxGetPr(prhs[0])[1], mxGetPr(prhs[0])[4], mxGetPr(prhs[0])[7],
		mxGetPr(prhs[0])[2], mxGetPr(prhs[0])[5], mxGetPr(prhs[0])[8]
		);	

	Vector3<double> theAxis;
	double theAngle;



	if ( fabs( M.Determinant() - 1.0 ) > 0.001 ) mexErrMsgTxt("3x3 matrix must represent a rotation i.e. det(R)==1");


	M.ToAxisAngle(theAxis, theAngle);


	plhs[0] = mxCreateDoubleMatrix(4, 1 , mxREAL);	    

	mxGetPr(plhs[0])[0] = theAxis.X();
	mxGetPr(plhs[0])[1] = theAxis.Y();
	mxGetPr(plhs[0])[2] = theAxis.Z();
	mxGetPr(plhs[0])[3] = theAngle;
	
}


//--------------------------------------------------------------
// mexFunction - Entry point from Matlab. From this C function,
//   simply call the C++ application function, above.
//--------------------------------------------------------------

	void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray  *prhs[] )
	{
		MatrixToAxisAngle(nlhs, plhs, nrhs, prhs);
	}
}






