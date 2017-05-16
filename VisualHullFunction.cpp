#include "VisualHullFunction.h"
#include "HomogeneousCoord.h"
#include "Matrix4x4.h"


VisualHullFunction::VisualHullFunction( const mxArray_tag* imgCell,
									  const mxArray_tag* internalsCell, 
									  const mxArray_tag* poseCell, 
									  const int numSilPerImg): numCams( mxGetNumberOfElements( poseCell ) )
{
	
	imgVec.resize( numCams );
	internalsVec.resize( numCams );
	poseVec.resize( numCams );
	
	
	//silPerImg is five for the double mirror setup 
	for(int camLoop = 0; camLoop < numCams; camLoop++)
	{
		imgVec[camLoop].Set( mxGetCell( imgCell, camLoop/numSilPerImg ) );
		
		poseVec[camLoop].FormFromMatlabArray( mxGetCell( poseCell, camLoop ) );
		
		internalsVec[camLoop].efl = mxGetPr( mxGetCell( internalsCell, camLoop ) )[0];
		internalsVec[camLoop].u0 = mxGetPr( mxGetCell( internalsCell, camLoop ) )[1];
		internalsVec[camLoop].v0 = mxGetPr( mxGetCell( internalsCell, camLoop ) )[2];
		
	}
	
}


VisualHullFunction::VisualHullFunction( const mxArray_tag* imgCell,
									   const mxArray_tag* internalsCell, 
									   const mxArray_tag* poseCell ) : numCams( mxGetNumberOfElements( imgCell ) )
{

	imgVec.resize( numCams );
		internalsVec.resize( numCams );
	poseVec.resize( numCams );

	
//mexPrintf("imgVec.size(): %d\n", imgVec.size() );
//mexPrintf("internalsVec.size(): %d\n", internalsVec.size() );
//mexPrintf("poseVec.size(): %d\n", poseVec.size() );
//mexPrintf("numCams: %d\n", numCams );


	for(int camLoop = 0; camLoop < numCams; camLoop++)
	{
		imgVec[camLoop].Set( mxGetCell( imgCell, camLoop ) );

		poseVec[camLoop].FormFromMatlabArray( mxGetCell( poseCell, camLoop ) );

		internalsVec[camLoop].efl = mxGetPr( mxGetCell( internalsCell, camLoop ) )[0];
		internalsVec[camLoop].u0 = mxGetPr( mxGetCell( internalsCell, camLoop ) )[1];
		internalsVec[camLoop].v0 = mxGetPr( mxGetCell( internalsCell, camLoop ) )[2];
	
	}

}


double VisualHullFunction::eval (double x, double y, double z)
{
	const HomogeneousCoord worldPoint( x, y, z, 1);

	
		//mexPrintf("[%g; %g; %g; %g]",
		//worldPoint.GetX(),
		//worldPoint.GetY(),
		//worldPoint.GetZ(),
		//worldPoint.GetW() );

	int camLoop = 0;
	bool insideHull = true; 
	
	while (  ( camLoop < numCams ) && insideHull  )
	{
		const HomogeneousCoord cameraPoint = poseVec[camLoop] * worldPoint;


		//mexPrintf("[%g; %g; %g; %g]",
		//cameraPoint.GetX(),
		//cameraPoint.GetY(),
		//cameraPoint.GetZ(),
		//cameraPoint.GetW() );

		const double uCoord = internalsVec[camLoop].efl * 
			( cameraPoint.GetX() / cameraPoint.GetZ() ) +
			internalsVec[camLoop].u0;

		const double vCoord = internalsVec[camLoop].efl * 
			( cameraPoint.GetY() / cameraPoint.GetZ() ) +
			internalsVec[camLoop].v0;



		//mexPrintf("camLoop: %d\n", camLoop);
		//mexPrintf("(%g, %g)\n", uCoord, vCoord );
			
		//mexPrintf("internalsVec[camLoop].efl %g\n", internalsVec[camLoop].efl );
		//mexPrintf("internalsVec[camLoop].u0 %g\n", internalsVec[camLoop].u0 );
		//mexPrintf("internalsVec[camLoop].v0 %g\n", internalsVec[camLoop].v0 );

		if (  imgVec[camLoop].GetIntensity( uCoord, vCoord ) == 0  )
			insideHull = false;
			
		camLoop++;
	}


	return insideHull?+1:-1;
    //double x2 = x*x, y2 = y*y, z2 = z*z;
    //double a = x2+y2+z2+(0.5*0.5)-(0.1*0.1);
    //return -(a*a-4.0*(0.5*0.5)*(y2+z2));
}




