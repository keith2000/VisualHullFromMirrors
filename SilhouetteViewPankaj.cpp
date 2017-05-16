#include <iostream>
#include <cmath>
#include <string>
#include <algorithm>
#include <set>
//#include <gsl/gsl_linalg.h>
#include "Wm3ContConvexHull2.h"
#include "Wm3LinearSystem.h"
#include "Wm3ContPointInPolygon2.h"
//#include "QHull2DWrapper.h" //for the qhull convex hull algorithm

////#define USE_WYKOBI  
//#ifdef USE_WYKOBI
//#include "wykobi.hpp"
//#include "wykobi_algorithm.hpp"
//#endif

#include "SilhouetteView.h"
//#include "TriangulateErrFn.h"
//#include "GslLevenbergMarquardt.h"
#include "SilFunctions.h"

#define for if(0); else for // Required for Windows for-loop scoping issues.
#define COMPLEXPRINT(str) mexPrintf("%% %s: %+g %+gi\n", #str, real(str), imag(str) );
#define DOUBLEPRINT(str) mexPrintf("%% %s: %g\n", #str, str );
#define BOOLPRINT(str) mexPrintf("%% %s: %s\n", #str, str?"true":"false" );
#define INTEGERPRINT(str) mexPrintf("%% %s: %d\n", #str, str );
#define P2DPRINT(str) mexPrintf(" %s= [%g; %g]\n", #str, str.X(), str.Y() );
#define P3DPRINT(str) mexPrintf("%% %s: [%g; %g; %g]\n", #str, str.X(), str.Y() , str.Z() );
#define M4PRINT(str) mexPrintf("%% %s= [[%g,%g,%g,%g];[%g,%g,%g,%g];[%g,%g,%g,%g];[%g,%g,%g,%g]]\n", #str, str(0,0), str(0,1), str(0,2), str(0,3), str(1,0), str(1,1), str(1,2), str(1,3), str(2,0), str(2,1), str(2,2), str(2,3), str(3,0), str(3,1), str(3,2), str(3,3) );
#define M3PRINT(str) mexPrintf("%% %s= [[%g,%g,%g];[%g,%g,%g];[%g,%g,%g]]\n", #str, str(0,0), str(0,1), str(0,2), str(1,0), str(1,1), str(1,2), str(2,0), str(2,1), str(2,2) );
#define MSASSERT(str) if (!(str)) throw(string("failed assertion:") + string(#str));

using namespace Wm3;
using namespace std;


SilhouetteView::SilhouetteView()
{
}

#ifdef MATLAB_MEX_FILE
SilhouetteView::SilhouetteView( const mxArray* matlabPoly )
{

	const int numVertices = int( mxGetN( matlabPoly ) );
	boundary.resize(numVertices);

	for( unsigned int vv = 0; vv < boundary.size(); ++vv )
	{
		boundary[vv] = Vector2d(
			mxGetPr( matlabPoly )[ vv*2 +0],
			mxGetPr( matlabPoly )[ vv*2 +1]);
	}

	SetUp();
}
#endif

void SilhouetteView::SetUp()
{
	FormConvexHull();

	//FormConvexHull();

	CreateModGradMap();	
	//ConvexSilhouetteCentroid( centrePoint );
	ConvexSilhouetteCentroid( centrePoint );
}

Wm3::Vector2d  SilhouetteView::VertexFromDirection( 
	const double xVal,  const double yVal  ) const
{
	const int ind = IndexFromDirection( xVal, yVal );
	return boundary[convHullIndVec[ind]];
}

int SilhouetteView::IndexFromDirection( 
									   const double xVal,  const double yVal  ) const
{
	const double mgrad = ModifiedGrad( xVal, yVal );

	//DOUBLEPRINT(mgrad);
	//boundary[vLoop].X(), boundary[vLoop].Y() );
	//	pointIndFromModGradMap.insert( make_pair( mgrad, vLoop) );


	map< double, int>::const_iterator mapCurrent;

	//for ( mapCurrent = pointIndFromModGradMap.begin();
	////	mapCurrent != pointIndFromModGradMap.end();
	//	mapCurrent++ )
	//	mexPrintf("%d: %g\n", (*mapCurrent).second, (*mapCurrent).first);


	mapCurrent = pointIndFromModGradMap.lower_bound( mgrad );

	if ( pointIndFromModGradMap.end() == mapCurrent   )
	{
		mapCurrent = pointIndFromModGradMap.begin();
		//mexPrintf("*END*\n");
	}

	//INTEGERPRINT( (*mapCurrent).second );
	//DOUBLEPRINT( (*mapCurrent).first );

	//mapCurrent = pointIndFromModGradMap.upper_bound( mgrad );
	//INTEGERPRINT( (*mapCurrent).second );
	//DOUBLEPRINT( (*mapCurrent).first );

	//int ind = (*mapCurrent).second - 1;
	//if (ind < 0)
	//	ind = convHullIndVec.size()-1;

	//const int ind = ((*mapCurrent).second + 0) % convHullIndVec.size();

	//return boundary[convHullIndVec[ind]];

	return (*mapCurrent).second;


}


void SilhouetteView::CreateNiBoundary()
{

	niBoundary.resize( boundary.size() );

	for (unsigned int vLoop = 0; vLoop < boundary.size(); vLoop++ )
	{
		niBoundary[vLoop] = 
			camera.ImageToNormalisedImage( boundary[vLoop] );
	}

}


void SilhouetteView::CreateOrthoBoundary( const Wm3::Vector3d centrePoint )
{

	if ( niBoundary.size() != boundary.size() )
		CreateNiBoundary();


	const Vector4d cenC = camera.pose * Vector4d(
		centrePoint.X(), 
		centrePoint.Y(), 
		centrePoint.Z(), 1 );

	orthoBoundary.resize( boundary.size() );

	for (unsigned int vLoop = 0; vLoop < boundary.size(); vLoop++ )
	{
		//	const Vector2d ni = camera.ImageToNormalisedImage( boundary[vLoop] );
		//	orthoBoundary[vLoop] = camera.ImageToNormalisedImage( boundary[vLoop] ) * cenC[2];

		orthoBoundary[vLoop] = niBoundary[vLoop] * cenC[2];
	}

}

void SilhouetteView::CreateConstDepthConvRim( 
	const Wm3::Vector3d centrePoint, 
	std::vector<Wm3::Vector3d> & rimVec ) const
{


	rimVec.resize( convHullIndVec.size() );

	Vector4d centrePointCam( 
		centrePoint.X(),
		centrePoint.Y(), 
		centrePoint.Z(), 1 );

	centrePointCam = camera.pose * centrePointCam;

	//DOUBLEPRINT( centrePointCam.Z() );
	//INTEGERPRINT(   convHullIndVec.size()  );

	for( unsigned int hullLoop = 0; hullLoop < convHullIndVec.size(); hullLoop++ )	
	{	


		const Vector3d pCam = centrePointCam.Z() * 
			Vector3d(
			niBoundary[ convHullIndVec[hullLoop] ].X(),
			niBoundary[ convHullIndVec[hullLoop] ].Y(), 1);

		camera.CameraToWorld( pCam,  rimVec[hullLoop] );

	}

}


void SilhouetteView::CreateModGradMap()
{

	for( unsigned int hullLoop = 0; hullLoop < convHullIndVec.size(); hullLoop++ )	
	{	

		const int nextInd = (hullLoop+1) % convHullIndVec.size();

		const Vector2d currentPoint = boundary[ convHullIndVec[hullLoop] ];
		const Vector2d nextPoint = boundary[ convHullIndVec[nextInd] ];

		const double mgrad = ModifiedGrad( 
			nextPoint.X() - currentPoint.X(), 
			nextPoint.Y() - currentPoint.Y() );

		//INTEGERPRINT(hullLoop);
		//INTEGERPRINT(nextInd);	
		//P2DPRINT(currentPoint);
		//P2DPRINT(nextPoint);
		//DOUBLEPRINT(nextPoint.X() - currentPoint.X());
		//DOUBLEPRINT(nextPoint.Y() - currentPoint.Y());
		//DOUBLEPRINT(mgrad);

		pointIndFromModGradMap.insert( make_pair( mgrad, nextInd) );


	}		



	//map< double, int>::const_iterator mapCurrent;
	//		for( mapCurrent = pointIndFromModGradMap.begin();
	//			mapCurrent != pointIndFromModGradMap.end(); mapCurrent++ )
	//		{		
	//			mexPrintf("<%0.20f, %3d>\n",  (*mapCurrent).first ,   (*mapCurrent).second );
	//		}

}

void SilhouetteView::CreateModGradLut()
{

	pointIndFromModGradLut.resize( convHullIndVec.size() );

	for( unsigned int hullLoop = 0; hullLoop < convHullIndVec.size(); hullLoop++ )	
	{	
		const int nextInd = (hullLoop+1) % convHullIndVec.size();

		const Vector2d currentPoint = boundary[ convHullIndVec[hullLoop] ];
		const Vector2d nextPoint = boundary[ convHullIndVec[nextInd] ];

		const double mgrad = ModifiedGrad( 
			nextPoint.X() - currentPoint.X(), 
			nextPoint.Y() - currentPoint.Y() );
		pointIndFromModGradLut[hullLoop] = mgrad;		
	}

	sort( pointIndFromModGradLut.begin(), 
		pointIndFromModGradLut.end() );
}



void SilhouetteView::ConvexSilhouetteCentroid(Wm3::Vector2d & centre) const
{

	const double area = SignedConvexArea();

	//%cx=\frac{1}{6A}/sum_0^{n-1} (x_i+x_{i+1})(x_i y_{i+1}-x_{i+1}y_i)
	//%cy=\frac{1}{6A}/sum_0^{n-1} (y_i+y_{i+1})(x_i y_{i+1}-x_{i+1}y_i)


	//A=(SignedArea(p));



	centre.X() = 0;
	centre.Y() = 0;

	for( unsigned int vLoop = 1; vLoop <= convHullIndVec.size(); vLoop++ )
	{

		const double factor = ( ( boundary[convHullIndVec[vLoop % convHullIndVec.size()]].X() *
			boundary[convHullIndVec[(vLoop+1) % convHullIndVec.size()]].Y() ) -
			( boundary[convHullIndVec[(vLoop+1) % convHullIndVec.size()]].X() *
			boundary[convHullIndVec[(vLoop) % convHullIndVec.size()]].Y() ) ); 


		centre.X() += 
			( boundary[convHullIndVec[vLoop % convHullIndVec.size()]].X() +
			boundary[convHullIndVec[(vLoop+1) % convHullIndVec.size()]].X() ) * factor;



		centre.Y() += 
			( boundary[convHullIndVec[vLoop % convHullIndVec.size()]].Y() +
			boundary[convHullIndVec[(vLoop+1) % convHullIndVec.size()]].Y() ) * factor;

	}

	centre.X() /= 6*area;
	centre.Y() /= 6*area;

}


double SilhouetteView::SignedConvexOrthoArea() const
{
	double soFar = 0;

	for( unsigned int vLoop = 1; vLoop <= convHullIndVec.size(); vLoop++ )
	{		
		soFar += 
			orthoBoundary[convHullIndVec[vLoop % convHullIndVec.size()]].X() *
			( orthoBoundary[convHullIndVec[(vLoop+1) % convHullIndVec.size()]].Y() -
			orthoBoundary[convHullIndVec[vLoop-1]].Y() );
	}

	//DOUBLEPRINT(soFar);

	return soFar/2;
}

double SilhouetteView::SignedConvexArea() const
{

	//     2 A(P) = sum_{i=0}^{n-1} ( x_i  (y_{i+1} - y_{i-1}) )


	double soFar = 0;

	for( unsigned int vLoop = 1; vLoop <= convHullIndVec.size(); vLoop++ )
	{

		//twiceArea+=vertexVec[v%numVertices].coord[0] * (vertexVec[(v+1)%numVertices].coord[1] - vertexVec[v-1].coord[1]);			


		//mexPrintf("%d %d %d\n",vLoop,
		//	(vLoop+1) % convHullIndVec.size(),
		//	(vLoop-1) % convHullIndVec.size() );

		soFar += 
			boundary[convHullIndVec[vLoop % convHullIndVec.size()]].X() *
			( boundary[convHullIndVec[(vLoop+1) % convHullIndVec.size()]].Y() -
			boundary[convHullIndVec[vLoop-1]].Y() );

	}


	return soFar/2;
}


double SilhouetteView::SignedArea() const
{
	double soFar = 0;
	for( unsigned int vLoop = 1; vLoop <= boundary.size(); vLoop++ )
	{
		soFar += 
			boundary[vLoop % boundary.size()].X() *
			( boundary[(vLoop+1) % boundary.size()].Y() - boundary[vLoop-1].Y() );
	}
	return soFar/2;
}



double SilhouetteView::AngleAtEpipole( const Wm3::Vector2d & epipole ) const
{	
	Vector2d tA, tB;	
	EpipolarTangencies( epipole, tA, tB );		
	Vector2d vA = (tA-epipole);
	vA = vA / vA.Length();	
	Vector2d vB = (tB-epipole);
	vB = vB / vB.Length();	
	const double dotProd =vA.Dot(vB);
	return acos(dotProd);	
}








double SilhouetteView::AngleAtEpipole(const SilhouetteView & viewOther, 
									  const Wm3::Matrix4d & RbtAdjust ) const
{
	Vector2d epipole;	
	camera.Epipole( viewOther.camera, RbtAdjust, epipole  );

	return AngleAtEpipole(epipole);
}










bool SilhouetteView::IsEpipoleInBoundRect( const SilhouetteView & viewOther, 
										  const Wm3::Matrix4d & RbtAdjust ) const
{

	Vector2d epipole;

	camera.Epipole( viewOther.camera, RbtAdjust, epipole  );


	//BOOLPRINT( IsEpipoleInBoundRect(epipole) );

	return IsEpipoleInBoundRect(epipole);

}


bool SilhouetteView::IsEpipoleInBoundRect( const Wm3::Vector2d & epipole ) const
{
	/*BOOLPRINT( epipole.X() > boundRect[0].X() );
	BOOLPRINT( epipole.Y() > boundRect[0].Y() );
	BOOLPRINT( epipole.X() < boundRect[1].X() );
	BOOLPRINT( epipole.Y() < boundRect[1].Y() );*/

	//P2DPRINT(epipole);
	//P2DPRINT(boundRect[0]);
	//P2DPRINT(boundRect[1]);

	return ( ( epipole.X() > boundRect[0].X() ) &&
		( epipole.Y() > boundRect[0].Y() ) &&
		( epipole.X() < boundRect[1].X() ) &&
		( epipole.Y() < boundRect[1].Y() ) );

}



bool SilhouetteView::IsTangency( const int vertexIndex, const Wm3::Vector2d & epipole ) const
{
	const int nextInd = (vertexIndex + 1) % convHullIndVec.size();
	const int prevInd = (vertexIndex + convHullIndVec.size() - 1) % convHullIndVec.size();

	const Vector2d vecEpi = epipole - boundary[convHullIndVec[vertexIndex]];

	const Vector2d vecNext = 
		boundary[convHullIndVec[nextInd]] - boundary[convHullIndVec[vertexIndex]];

	const Vector2d vecPrev = 
		boundary[convHullIndVec[prevInd]] - boundary[convHullIndVec[vertexIndex]];

	//if we are at a tangency, then vecNext and vecPrev will be on the same side of vecEpi

	const Vector2d unitPerp = vecEpi.UnitPerp();

	return unitPerp.Dot( vecNext ) * unitPerp.Dot( vecPrev ) > 0;
	//if they have the same sign, then they are on the same side

}

void SilhouetteView::TangenciesFromMap( const Wm3::Vector2d & epipole,
									   Wm3::Vector2d & tRight, Wm3::Vector2d & tLeft ) const
{

	//epipole is assumed to not lie inside -- otherwise infinite loop!
	//from the epipoles point of view, 
	//	tRight is the tangency to the right
	//	tLeft is the tangency to the left



	long count = 0;
	const long maxCount = 40;//20;

	bool stillLooking;
	Vector2d approxTarget;

	stillLooking = true;
	approxTarget = centrePoint;

	while (stillLooking)
	{
		count++;
		const int vertexIndex = IndexFromDirection( 
			epipole.X()-approxTarget.X(),  
			epipole.Y()-approxTarget.Y() );
		tRight = boundary[convHullIndVec[vertexIndex]];
		if ( IsTangency( vertexIndex, epipole ) )		
			stillLooking = false;		
		else		
			approxTarget = tRight;	

		if ( count > maxCount )
		{
			mexPrintf("count:%d\n", count);
			mexPrintf("poly=[ ");
			for( unsigned int jj=0; jj < convHullIndVec.size();++jj )
				mexPrintf("[%0.20f;%0.20f] ", boundary[convHullIndVec[jj]].X(), 
				boundary[convHullIndVec[jj]].Y() );
			mexPrintf("];\n");

			mexPrintf("tRight = [%0.20f;%0.20f]; epi = [%0.20f;%0.20f];\n",
				tRight.X(), tRight.Y(), epipole.X(), epipole.Y() );
			mexPrintf("figure, hold on, ShowPoly(poly), plot2D(poly, '.k');");
			mexPrintf("plot2D([epi, tRight], '*r');");
			mexPrintf("plot2D([epi, tRight+1.33*(tRight-epi)], '-r');");
			mexPrintf("plot2D(epi, '*m', 'MarkerSize', 20);\n");

			throw( string("too many tries 234edw (check for collinear polygon vertices--these are not allowed)") );
		}

	}


	stillLooking = true;
	approxTarget = centrePoint;
	while (stillLooking)
	{
		count++;
		const int vertexIndex = IndexFromDirection( 
			approxTarget.X() - epipole.X(),  
			approxTarget.Y() - epipole.Y()  );
		tLeft = boundary[convHullIndVec[vertexIndex]];


		//P2DPRINT(tLeft);

		if ( IsTangency( vertexIndex, epipole ) )		
			stillLooking = false;		
		else		
			approxTarget = tLeft;		

		if ( count > maxCount )
		{
			mexPrintf("poly=[ ");
			for( unsigned int jj=0; jj < convHullIndVec.size();++jj )
				mexPrintf("[%0.20f;%0.20f] ", boundary[convHullIndVec[jj]].X(), 
				boundary[convHullIndVec[jj]].Y() );
			mexPrintf("];\n");

			mexPrintf("tLeft = [%0.20f;%0.20f]; epi = [%0.20f;%0.20f];\n",
				tLeft.X(), tLeft.Y(), epipole.X(), epipole.Y() );
			mexPrintf("figure, hold on, ShowPoly(poly), plot2D(poly, '.k');");
			mexPrintf("plot2D([epi, tLeft], '*-r');");
			mexPrintf("plot2D(epi, '*m', 'MarkerSize', 20);\n");

			mexPrintf("count = %d;\n", count);


			const int nextInd = (vertexIndex + 1) % convHullIndVec.size();
			const int prevInd = (vertexIndex + convHullIndVec.size() - 1) % convHullIndVec.size();

			INTEGERPRINT(nextInd);
			INTEGERPRINT(prevInd);


			const Vector2d vecEpi = epipole - boundary[convHullIndVec[vertexIndex]];
			const Vector2d vecNext = boundary[convHullIndVec[nextInd]] - boundary[convHullIndVec[vertexIndex]];
			const Vector2d vecPrev = boundary[convHullIndVec[prevInd]] - boundary[convHullIndVec[vertexIndex]];
			const Vector2d unitPerp = vecEpi.UnitPerp();
			const double dotProduct =  unitPerp.Dot( vecNext ) * unitPerp.Dot( vecPrev );

			const Vector2d tangPoint = boundary[convHullIndVec[vertexIndex]];
			P2DPRINT(  tangPoint  );
			P2DPRINT(vecEpi);
			P2DPRINT(vecNext);
			P2DPRINT(vecPrev);
			DOUBLEPRINT(dotProduct);


			const double mgrad = ModifiedGrad( approxTarget.X() - epipole.X(),  
				approxTarget.Y() - epipole.Y() );

			DOUBLEPRINT(mgrad);

			map< double, int>::const_iterator mapCurrent;
			for( mapCurrent = pointIndFromModGradMap.begin();
				mapCurrent != pointIndFromModGradMap.end(); mapCurrent++ )
			{		
				mexPrintf("<%0.20f, %3d>\n",  (*mapCurrent).first ,   (*mapCurrent).second );
			}

			throw( string("too many tries 1dwfwe (collinear vertices?)") );
		}
	}


	//mexPrintf("poly=[ ");
	//for( unsigned int jj=0; jj < convHullIndVec.size();++jj )
	//	mexPrintf("[%g;%g] ", boundary[convHullIndVec[jj]].X(), 
	//	boundary[convHullIndVec[jj]].Y() );
	//mexPrintf("];\n");

}

void SilhouetteView::PrintOrthoBoundary() const
{
	mexPrintf("poly=[ ");
	for( unsigned int jj=0; jj < orthoBoundary.size();++jj )
		mexPrintf("[%g;%g] ", 
		orthoBoundary[jj].X(), orthoBoundary[jj].Y() );
	//	boundary[convHullIndVec[jj]].Y() );
	mexPrintf("];\n");
}

void SilhouetteView::PrintConvBoundary() const
{
	mexPrintf("poly=[ ");
	for( unsigned int jj=0; jj < convHullIndVec.size();++jj )
		mexPrintf("[%g;%g] ", 
		boundary[convHullIndVec[jj]].X(),
		boundary[convHullIndVec[jj]].Y() );

	mexPrintf("];\n");
}

void SilhouetteView::EpipolarTangencies( const Wm3::Vector2d & epipole,
										Wm3::Vector2d & tA, Wm3::Vector2d & tB ) const
{

	tA = boundary[ convHullIndVec[0] ];
	tB = tA;

	for( unsigned int kLoop = 1; kLoop < convHullIndVec.size(); kLoop++) //start from 1 since 0 has just been dealt with
	{
		const Vector2d & current = boundary[ convHullIndVec[kLoop] ];

		if  ( IsFurtherClockwise( tA-epipole, current-epipole ) )
		{
			tA = current;
		}

		if  ( IsFurtherClockwise( current-epipole, tB-epipole ) )
		{
			tB = current;
		}

	}

}



bool SilhouetteView::IsFurtherClockwise(const Wm3::Vector2d& A, const Wm3::Vector2d& B) const
{
	if (A.Y() * B.Y() > 0) //both on the same side of the epipole
	{
		return (A.X()/A.Y() < B.X()/B.Y());
	}

	if (A.Y() * B.Y() == 0) //at least one slope is infinity or -infinity
	{

		if (A.Y() == 0)
		{
			if (A.X() > 0)
			{
				return B.Y() < 0;
			}
			else
			{
				return B.Y() > 0;
			}
		}
		else
		{
			if (B.X() > 0)
			{
				return A.Y() > 0;
			}
			else
			{
				return A.Y() < 0;
			}
		}
	}

	return (A.X()/A.Y() > B.X()/B.Y());		
}


//void SilhouetteView::LmTriangulate(
//								   const Wm3::Vector2d & imgPointA, 
//								   const Wm3::Vector2d & imgPointB, 
//								   const VcalCamera & cameraA, 
//								   const VcalCamera & cameraB, 
//								   Wm3::Vector3d & approxPoint ) const
//{
//
//	ApproxTriangulate( imgPointA, imgPointB,  cameraA, cameraB, approxPoint);
//	//initial estimate
//
//	vector<Vector2d> imgCentroidVec( 2 );
//	vector<VcalCamera> camVec( 2 );
//
//	imgCentroidVec[0] = imgPointA;
//	camVec[0] = cameraA;
//
//	imgCentroidVec[1] = imgPointB;
//	camVec[1] = cameraB;
//
//
//	TriangulateErrFn ef( &imgCentroidVec, &camVec );
//
//	GslLevenbergMarquardt lm( &ef );	
//
//
//	const double tol = 1e-7;
//	const long maxIterations = 3;
//
//	lm.SetTolF( tol );	
//	lm.SetMaxIter( maxIterations );
//
//
//	vector<double> initVec(3);
//	initVec[0] = approxPoint.X();
//	initVec[1] = approxPoint.Y();
//	initVec[2] = approxPoint.Z();
//
//	vector<double> optVec(3);
//	lm.Minimize( initVec, optVec );
//
//
//	approxPoint.X() = optVec[0];
//	approxPoint.Y() = optVec[1];
//	approxPoint.Z() = optVec[2];
//
//
//
//
//}
//
//
//
//
//void SilhouetteView::ApproxTriangulate(
//									   const Wm3::Vector2d & imgPointA, 
//									   const Wm3::Vector2d & imgPointB, 
//									   const VcalCamera & cameraA, 
//									   const VcalCamera & cameraB, 
//									   Wm3::Vector3d & approxPoint ) const
//{
//
//	vector<double> bb( 2*2 );	 
//	GMatrix<double> AA( 2*2, 3 );
//	Vector3d rowVec;
//	Vector2d niPoint;		
//	Matrix3d R;	
//	Vector3d t;	
//
//	cameraA.CreateR(R);			
//	cameraA.Create_t(t);		
//	niPoint = cameraA.ImageToNormalisedImage(imgPointA);
//	rowVec = R.GetRow(0) - niPoint.X() * R.GetRow(2);		
//	for(int cc=0;cc<3;cc++)
//		AA(0 * 2 + 0, cc) = rowVec[cc];		
//	rowVec = R.GetRow(1) - niPoint.Y() * R.GetRow(2);
//	for(  int cc=0;cc<3;cc++)
//		AA(0 * 2 + 1, cc) = rowVec[cc];		
//	bb[0 * 2 + 0] = niPoint.X() * t.Z() - t.X();
//	bb[0 * 2 + 1] = niPoint.Y() * t.Z() - t.Y();
//
//
//	cameraB.CreateR(R);		
//	cameraB.Create_t(t);
//	niPoint = cameraB.ImageToNormalisedImage(imgPointB);
//	rowVec = R.GetRow(0) - niPoint.X() * R.GetRow(2);		
//	for(  int cc=0;cc<3;cc++)
//		AA(1 * 2 + 0, cc) = rowVec[cc];		
//	rowVec = R.GetRow(1) - niPoint.Y() * R.GetRow(2);
//	for(  int cc=0;cc<3;cc++)
//		AA(1 * 2 + 1, cc) = rowVec[cc];		
//	bb[1 * 2 + 0] = niPoint.X() * t.Z() - t.X();
//	bb[1 * 2 + 1] = niPoint.Y() * t.Z() - t.Y();
//
//	/*/////////////
//	mexPrintf("AA = [ ");
//	for( unsigned int cc=0; cc<3; cc++)
//	mexPrintf("[%g;%g;%g;%g] ", 
//	AA(0, cc),AA(1, cc),AA(2, cc),AA(3, cc) );
//	mexPrintf(" ]\n");
//
//	mexPrintf("bb = [%g;%g;%g;%g]\n ", bb[0], bb[1], bb[2], bb[3] );
//	//for( unsigned int rr=0; rr<4; rr++)
//	//	mexPrintf("b:%g\n", bb[rr]  );
//	/*/////////////
//	gsl_matrix * AAA = gsl_matrix_alloc ( 4, 4 );
//
//	for ( int r = 0; r < 4; r++)
//		for ( int c = 0; c < 3; c++)
//			gsl_matrix_set(AAA, r, c, AA(r,c) );
//
//
//	gsl_matrix_set(AAA, 0, 3, 0 );
//	gsl_matrix_set(AAA, 1, 3, 0 );
//	gsl_matrix_set(AAA, 2, 3, 0 );
//	gsl_matrix_set(AAA, 3, 3, 0 );
//
//
//	gsl_matrix * VVV = gsl_matrix_alloc (  4, 4  );
//	gsl_vector * sss = gsl_vector_alloc ( 4 );
//	gsl_vector * work= gsl_vector_alloc ( 4 );
//
//
//	/*/////////////
//	mexPrintf("AAA = [ ");
//	for( unsigned int cc=0; cc<4; cc++)
//	mexPrintf("[%g;%g;%g;%g] ", 
//	gsl_matrix_get(AAA, 0, cc),
//	gsl_matrix_get(AAA, 1, cc),
//	gsl_matrix_get(AAA, 2, cc),
//	gsl_matrix_get(AAA, 3, cc) );				
//	mexPrintf(" ]\n");
//	/*/////////////
//
//	gsl_linalg_SV_decomp (AAA, VVV, sss, work);
//
//
//	//gsl_vector * bbb= gsl_vector_alloc ( 4 );
//
//	gsl_vector * bbb= gsl_vector_alloc ( 4 );
//	for( unsigned int jj = 0; jj < 4;++jj)
//		gsl_vector_set( bbb, jj, bb[jj] );
//
//
//	gsl_vector * xxx = gsl_vector_alloc ( 4 );
//	gsl_linalg_SV_solve (AAA, VVV, sss, bbb, xxx );
//
//	approxPoint.X() = gsl_vector_get(xxx, 0);
//	approxPoint.Y() = gsl_vector_get(xxx, 1);
//	approxPoint.Z() = gsl_vector_get(xxx, 2);
//
//	/*/////////////
//	mexPrintf("UUU = [ ");
//	for( unsigned int cc=0; cc<4; cc++)
//	mexPrintf("[%g;%g;%g;%g] ", 
//	gsl_matrix_get(AAA, 0, cc),
//	gsl_matrix_get(AAA, 1, cc),
//	gsl_matrix_get(AAA, 2, cc),
//	gsl_matrix_get(AAA, 3, cc) );				
//	mexPrintf(" ]\n");
//	mexPrintf("VVV = [ ");
//	for( unsigned int cc=0; cc<4; cc++)
//	mexPrintf("[%g;%g;%g;%g] ", 
//	gsl_matrix_get(VVV, 0, cc),
//	gsl_matrix_get(VVV, 1, cc),
//	gsl_matrix_get(VVV, 2, cc),
//	gsl_matrix_get(VVV, 3, cc) );				
//	mexPrintf(" ]\n");
//
//	mexPrintf("sss = [%g;%g;%g;%g]\n", 
//	gsl_vector_get(sss, 0),
//	gsl_vector_get(sss, 1),
//	gsl_vector_get(sss, 2),
//	gsl_vector_get(sss, 3) );
//
//	mexPrintf("xxx = [%g;%g;%g;%g]\n", 
//	gsl_vector_get(xxx, 0),
//	gsl_vector_get(xxx, 1),
//	gsl_vector_get(xxx, 2),
//	gsl_vector_get(xxx, 3) );
//	/*/////////////
//	gsl_matrix_free ( AAA );
//	gsl_matrix_free ( VVV );
//	gsl_vector_free ( sss );
//	gsl_vector_free ( work );
//	gsl_vector_free ( bbb );
//	gsl_vector_free ( xxx );
//
//
//
//	/*///////////////
//	LinearSystemd linsys;
//
//	if ( !linsys.Solve( AA, &(bb[0]), &(approxPoint[0]) ) )
//	throw string("Failed to solve linear system.");	
//	/*///////////////
//}
//
//void SilhouetteView::EstFrontierPoints(const SilhouetteView & viewOpp,
//									   const Wm3::Matrix4d & RbtAdjust,
//									   Wm3::Vector3d & frontierPointA,
//									   Wm3::Vector3d & frontierPointB) const
//{
//
//	Vector2d epiOnThis;
//	camera.Epipole( viewOpp.camera, RbtAdjust, epiOnThis );
//
//	Vector2d epiOnOpp;
//	viewOpp.camera.Epipole( camera, RbtAdjust.Inverse(), epiOnOpp );
//
//	Vector2d tThisA, tThisB;
//	//Vector2d tThisANew, tThisBNew;
//	//TangenciesFromMap( epiOnThis, tThisANew, tThisBNew );
//	//P2DPRINT(epiOnThis);
//	//P2DPRINT(tThisANew);
//	//P2DPRINT(tThisBNew);
//	EpipolarTangencies(epiOnThis, tThisA, tThisB );
//	//P2DPRINT(tThisA);
//	//P2DPRINT(tThisB);
//
//
//
//	Vector2d tOppA, tOppB;
//	viewOpp.EpipolarTangencies(epiOnOpp, tOppA, tOppB );
//
//
//
//
//
//	VcalCamera adjustedCamera = viewOpp.camera;
//	adjustedCamera.pose = 
//		( RbtAdjust * viewOpp.camera.pose.Inverse() ).Inverse();
//
//	//mexPrintf("---------------------\n");
//	//	M4PRINT(adjustedCamera.pose);
//	//	M4PRINT(camera.pose);
//
//	//if ( camera.AreFacing( adjustedCamera.pose ) )
//
//
//	//	const Matrix4d otherPose = 	
//	//	Sil::PoseInvert( RbtAdjust * 
//	//	Sil::PoseInvert( viewOpp.camera.pose) ); 
//
//
//
//	if ( Sil::IsBehind( camera.pose, adjustedCamera.pose  ) ==
//		Sil::IsBehind( adjustedCamera.pose, camera.pose ) )
//
//	{
//		//		mexPrintf("facing\n");
//		swap( tOppA, tOppB );
//	}
//	else
//	{
//		//		mexPrintf("one's behind the other\n");
//		//swap( tOppA, tOppB ); //shouldn't be swapping
//	}
//
//	/*
//	P2DPRINT(tThisA);
//	P2DPRINT(tThisB);
//	P2DPRINT(tOppA);
//	P2DPRINT(tOppB);
//	*/
//	//LmTriangulate
//	LmTriangulate( 
//		tThisA, tOppA, camera, adjustedCamera, frontierPointA);
//
//	Vector2d projThisA;
//	camera.WorldToImage(frontierPointA, projThisA);
//
//	//P3DPRINT(frontierPointA);
//	//	P2DPRINT(Vector2d(projThisA-tThisA));
//
//	/////////
//	//Vector3d frontierPointAB;
//	//ApproxTriangulate( 
//	//tThisA, tOppB, camera, adjustedCamera, frontierPointAB);
//
//	//Vector2d projThisAB;
//	//camera.WorldToImage(frontierPointAB, projThisAB);
//	//	P2DPRINT(Vector2d(projThisAB-tThisA));
//	/////////
//
//	Vector2d projOppA;	
//	adjustedCamera.WorldToImage(frontierPointA, projOppA);
//	//P2DPRINT(Vector2d(projOppA-tOppA));
//
//	//P3DPRINT(frontierPointA);
//	//ApproxTriangulate
//	LmTriangulate( 
//		tThisB, tOppB, camera, adjustedCamera, frontierPointB);
//
//	//P3DPRINT(frontierPointB);
//
//
//}
//
//
//void SilhouetteView::FrontierErr(const SilhouetteView & viewOpp,
//								 const Wm3::Matrix4d & RbtAdjust,
//								 const Wm3::Vector3d & frontierPointA,
//								 const Wm3::Vector3d & frontierPointB,
//								 double * resVec ) const
//{
//
//	Vector2d epiOnThis;	
//	camera.Epipole( viewOpp.camera, RbtAdjust, epiOnThis );
//
//	Vector2d tThisA, tThisB;	
//	EpipolarTangencies(epiOnThis, tThisA, tThisB );
//
//	Vector2d frontierProjA, frontierProjB;	
//	camera.WorldToImage( frontierPointA, frontierProjA );
//	camera.WorldToImage( frontierPointB, frontierProjB );
//
//	const double distA = 
//		(frontierProjA.X() - tThisA.X())*(frontierProjA.X() - tThisA.X()) +
//		(frontierProjA.Y() - tThisA.Y())*(frontierProjA.Y() - tThisA.Y());
//
//	//DOUBLEPRINT(sqrt(distA));
//
//	const double distB = 
//		(frontierProjA.X() - tThisB.X())*(frontierProjA.X() - tThisB.X()) +
//		(frontierProjA.Y() - tThisB.Y())*(frontierProjA.Y() - tThisB.Y());
//
//	//DOUBLEPRINT(sqrt(distB));
//
//	if ( distB < distA)
//		swap(tThisA, tThisB);
//
//
//	resVec[0] = frontierProjA.X() - tThisA.X();
//	resVec[1] = frontierProjA.Y() - tThisA.Y();
//	resVec[2] = frontierProjB.X() - tThisB.X();
//	resVec[3] = frontierProjB.Y() - tThisB.Y();
//
//	/*
//	DOUBLEPRINT(  frontierProjA.X() );
//	DOUBLEPRINT(  tThisA.X() );
//
//	DOUBLEPRINT(resVec[0]);
//	DOUBLEPRINT(resVec[1]);
//	DOUBLEPRINT(resVec[2]);
//	DOUBLEPRINT(resVec[3]);
//	*/
//
//}
//
//
void SilhouetteView::EtSimilarityResiduals(const SilhouetteView & viewOpp,
										   const Wm3::Matrix4d & RbtScaleAdjust,
										   bool & isAnEpipoleInside,
										   double * resVec ) const

{


	//the method for pose estimation doesn't work for RbtScaleAdjust so 
	//we'll use this function instead.
	Vector2d epiOnThis, epiOnOpp;
	//EtSimilarityResiduals
	camera.Epipole( viewOpp.camera, RbtScaleAdjust, epiOnThis );
	viewOpp.camera.Epipole( camera, RbtScaleAdjust.Inverse(), epiOnOpp );
	isAnEpipoleInside =
		IsEpipoleInBoundRect(epiOnThis) ||
		viewOpp.IsEpipoleInBoundRect(epiOnOpp);
	//EtSimilarityResiduals
	if ( isAnEpipoleInside )
	{
		resVec[0] = -999;
		resVec[1] = -998;
		resVec[2] = -997;
		resVec[3] = -996; //thse should never be used as they are flagged by isAnEpipoleInside as being invalid
	}
	else
	{
		Vector2d  tThisA, tThisB, tOppA, tOppB;
		TangenciesFromMap(epiOnThis, tThisA, tThisB );
		viewOpp.TangenciesFromMap(epiOnOpp, tOppA, tOppB );
		//EtSimilarityResiduals
		const Matrix4d otherPose =
			( RbtScaleAdjust *  viewOpp.camera.pose.Inverse() ).Inverse();
		const Matrix4d Rt =
			camera.pose * RbtScaleAdjust *
			( viewOpp.camera.pose.Inverse() );
		//EtSimilarityResiduals
		const Matrix3d tx(
			0, -Rt(2,3), +Rt(1,3),
			+Rt(2,3),        0, -Rt(0,3),
			-Rt(1,3), +Rt(0,3),       0 );
		//EtSimilarityResiduals
		const Matrix3d R(
			Rt(0,0), Rt(0,1), Rt(0,2),
			Rt(1,0), Rt(1,1), Rt(1,2),
			Rt(2,0), Rt(2,1), Rt(2,2) );
		//EtSimilarityResiduals
		const Matrix3d E_prime = tx * R; //essential matrix	from opp to this

		const Matrix3d E = E_prime.Transpose();//essential matrix from this to opp

		Matrix3d thisInvK, oppInvK;
		camera.CreateInvK( thisInvK );
		viewOpp.camera.CreateInvK( oppInvK );
		//EtSimilarityResiduals
		Matrix3d thisK, oppK;
		camera.CreateK( thisK );
		viewOpp.camera.CreateK( oppK );
		//EtSimilarityResiduals
		const Matrix3d F_prime =
			(thisK.Inverse()).Transpose() * E_prime * (oppK.Inverse());

		const Matrix3d F = (oppK.Inverse()).Transpose() * E * (thisK.Inverse());

		Vector3d tOppHomogA( tOppA.X(),  tOppA.Y(),  1 );
		const Vector3d tThisHomogA( tThisA.X(),  tThisA.Y(),  1 );
		Vector3d tOppHomogB( tOppB.X(),  tOppB.Y(),  1 );
		const Vector3d tThisHomogB( tThisB.X(),  tThisB.Y(),  1 );
		//EtSimilarityResiduals		

		if ( fabs( tOppHomogB.Dot( F * tThisHomogA) ) < 
			fabs( tOppHomogA.Dot( F * tThisHomogA) ) )
			swap( tOppHomogA, tOppHomogB );
		//EtSimilarityResiduals
		Vector3d lineDirection, perpLineThruPoint;
		//we are only considering lines projected onto this image
		const Vector3d lineA = F_prime * tOppHomogA;
		lineDirection = Vector3d( lineA.X(), lineA.Y(), 0 );
		perpLineThruPoint =  lineDirection.Cross( tThisHomogA );
		Vector3d pA = lineA.Cross(perpLineThruPoint);
		pA = pA/pA.Z();
		const Vector3d lineB = F_prime * tOppHomogB;
		lineDirection = Vector3d( lineB.X(), lineB.Y(), 0 );
		perpLineThruPoint =  lineDirection.Cross( tThisHomogB );
		Vector3d pB = lineB.Cross(perpLineThruPoint);
		pB = pB/pB.Z();
		resVec[0] = pA.X() - tThisA.X();
		resVec[1] = pA.Y() - tThisA.Y();
		resVec[2] = pB.X() - tThisB.X();
		resVec[3] = pB.Y() - tThisB.Y();
		//mexPrintf("%g %g %g %g\n",
		//resVec[0],
		//resVec[1],
		//resVec[2],
		//resVec[3] );
	}

}






void SilhouetteView::EtResiduals(const SilhouetteView & viewOpp,
								 const Wm3::Matrix4d & RbtAdjust,
								 bool & isAnEpipoleInside,
								 double * resVec ) const
{

	//mexPrintf("#@");
	Vector2d epiOnThis, epiOnOpp;

	camera.Epipole( viewOpp.camera, RbtAdjust, epiOnThis );
	viewOpp.camera.Epipole( camera, RbtAdjust.Inverse(), epiOnOpp );

	isAnEpipoleInside = 
		IsEpipoleInBoundRect(epiOnThis) || 
		viewOpp.IsEpipoleInBoundRect(epiOnOpp);


	//BOOLPRINT("isAnEpipoleInside");

	//if (isAnEpipoleInside)
	//	mexPrintf("w");
	//else
	//	mexPrintf("#@");


	if ( isAnEpipoleInside )
	{
		resVec[0] = -999;
		resVec[1] = -998;
		resVec[2] = -997;
		resVec[3] = -996; //thse should never be used as they are flagged by isAnEpipoleInside as being invalid
	}
	else
	{
		Vector2d  tThisA, tThisB, tOppA, tOppB;

		TangenciesFromMap(epiOnThis, tThisA, tThisB );	
		viewOpp.TangenciesFromMap(epiOnOpp, tOppA, tOppB );

		//		P2DPRINT(tThisA);
		//P2DPRINT(tThisB);
		//		P2DPRINT(tOppA);
		//P2DPRINT(tOppB);

		//if ( camera.AreFacing( 
		//Sil::PoseInvert( RbtAdjust * 
		//Sil::PoseInvert( viewOpp.camera.pose) ) ) )


		//		const Matrix4d otherPose = 	
		//			Sil::PoseInvert( RbtAdjust * 
		//			Sil::PoseInvert( viewOpp.camera.pose) ); 

		const Matrix4d otherPose = 	
			( RbtAdjust *  viewOpp.camera.pose.Inverse() ).Inverse(); 


		/*	const bool behindA =  Sil::IsBehind( camera.pose, otherPose  );
		const bool behindB = Sil::IsBehind( otherPose, camera.pose );
		BOOLPRINT(behindA);
		BOOLPRINT(behindB);*/


		if ( Sil::IsBehind( camera.pose, otherPose  ) ==
			Sil::IsBehind( otherPose, camera.pose ) )
		{
			//if cameras are facing then they are looking from opposite sides
			//or if they are both behind each other
			swap(tOppA, tOppB);
			//mexPrintf("facing");
		}

		//const Matrix4d RtDeleteMe = camera.pose * RbtAdjust *
		//	Sil::PoseInvert( viewOpp.camera.pose );


		const Matrix4d Rt = 
			camera.pose * RbtAdjust *
			( viewOpp.camera.pose.Inverse() );

		//M4PRINT(Rt);
		//const Matrix4d RtDeleteMeToo =  Rt-RtDeleteMe;
		//M4PRINT(RtDeleteMeToo);


		const Matrix3d tx( 
			0, -Rt(2,3), +Rt(1,3),
			+Rt(2,3),        0, -Rt(0,3),
			-Rt(1,3), +Rt(0,3),       0 );

		const Matrix3d R( 
			Rt(0,0), Rt(0,1), Rt(0,2),
			Rt(1,0), Rt(1,1), Rt(1,2),
			Rt(2,0), Rt(2,1), Rt(2,2) );

		const Matrix3d E_prime = tx * R; //essential matrix	from opp to this

		//const Matrix3d E = E_prime.Transpose();//essential matrix from this to opp



		Matrix3d thisInvK, oppInvK;
		camera.CreateInvK( thisInvK );
		viewOpp.camera.CreateInvK( oppInvK );


		Matrix3d thisK, oppK;
		camera.CreateK( thisK );
		viewOpp.camera.CreateK( oppK );

		//Matrix3d F = (oppK.Inverse()).Transpose() * E * (thisK.Inverse());

		//Matrix3d F_prime2 = thisInvK.Transpose() * E_prime * oppInvK;

		Matrix3d F_prime = 
			(thisK.Inverse()).Transpose() * E_prime * (oppK.Inverse());

		//M3PRINT(F_prime);
		//M3PRINT(F_prime2);
		//M3PRINT(oppInvK);
		//M3PRINT(oppK.Inverse());

		const Vector3d tOppHomogA( tOppA.X(),  tOppA.Y(),  1 );
		const Vector3d tThisHomogA( tThisA.X(),  tThisA.Y(),  1 );	
		const Vector3d tOppHomogB( tOppB.X(),  tOppB.Y(),  1 );
		const Vector3d tThisHomogB( tThisB.X(),  tThisB.Y(),  1 );

		Vector3d lineDirection, perpLineThruPoint;

		//we are only considering lines projected onto this image
		const Vector3d lineA = F_prime * tOppHomogA;
		lineDirection = Vector3d( lineA.X(), lineA.Y(), 0 );

		//M3PRINT(F_prime);
		//P3DPRINT(lineA);
		//P3DPRINT(tOppHomogA);

		perpLineThruPoint =  lineDirection.Cross( tThisHomogA );
		Vector3d pA = lineA.Cross(perpLineThruPoint);
		pA = pA/pA.Z();

		//P2DPRINT(pA);

		const Vector3d lineB = F_prime * tOppHomogB;	
		lineDirection = Vector3d( lineB.X(), lineB.Y(), 0 );
		perpLineThruPoint =  lineDirection.Cross( tThisHomogB );
		Vector3d pB = lineB.Cross(perpLineThruPoint);
		pB = pB/pB.Z();

		//P2DPRINT(pB);
		resVec[0] = pA.X() - tThisA.X();
		resVec[1] = pA.Y() - tThisA.Y();
		resVec[2] = pB.X() - tThisB.X();
		resVec[3] = pB.Y() - tThisB.Y();


		/*mexPrintf("%g %g %g %g\n",
		resVec[0],
		resVec[1],
		resVec[2],
		resVec[3] );*/
	}

}





void SilhouetteView::EtResiduals(const SilhouetteView & viewOpp,
								 const Wm3::Matrix4d & RbtAdjust,
								 double * resVec ) const
{

	Vector2d epiOnThis, epiOnOpp, tThisA, tThisB, tOppA, tOppB;

	camera.Epipole( viewOpp.camera, RbtAdjust, epiOnThis );
	viewOpp.camera.Epipole( camera, RbtAdjust.Inverse(), epiOnOpp );

	//TangenciesFromMap( epiOnThis, tThisA, tThisB );
	//P2DPRINT(tThisA);
	//P2DPRINT(tThisB);
	//P2DPRINT(epiOnThis);
	//P2DPRINT(tThisA);
	//P2DPRINT(tThisB);
	//EpipolarTangencies(epiOnThis, tThisA, tThisB );
	//	EpipolarTangencies(epiOnThis, tThisA, tThisB );


#define ORIG_TANG_METHOD
#ifdef ORIG_TANG_METHOD
	EpipolarTangencies(epiOnThis, tThisA, tThisB );	
	viewOpp.EpipolarTangencies(epiOnOpp, tOppA, tOppB );
#else
	//TangenciesFromMap(epiOnThis, tThisA, tThisB );	
	//viewOpp.TangenciesFromMap(epiOnOpp, tOppA, tOppB );
#endif
	//			P2DPRINT(tThisA);
	//	P2DPRINT(tThisB);
	//			P2DPRINT(tOppA);
	//P2DPRINT(tOppB);

	//	viewOpp.EpipolarTangencies(epiOnOpp, tOppA, tOppB );

	const Matrix4d Rt = camera.pose * RbtAdjust *
		viewOpp.camera.pose.Inverse();

	//P3DPRINT(tOppHomogA);

	const Matrix3d tx( 
		0, -Rt(2,3), +Rt(1,3),
		+Rt(2,3),        0, -Rt(0,3),
		-Rt(1,3), +Rt(0,3),       0 );

	const Matrix3d R( 
		Rt(0,0), Rt(0,1), Rt(0,2),
		Rt(1,0), Rt(1,1), Rt(1,2),
		Rt(2,0), Rt(2,1), Rt(2,2) );

	const Matrix3d E_prime = tx * R; //essential matrix	from opp to this

	const Matrix3d E = E_prime.Transpose();//essential matrix from this to opp


	Matrix3d thisK, oppK;

	camera.CreateK( thisK );

	viewOpp.camera.CreateK( oppK );

	Matrix3d F = (oppK.Inverse()).Transpose() * E * (thisK.Inverse());

	Matrix3d F_prime = 
		(thisK.Inverse()).Transpose() * E_prime * (oppK.Inverse());



	Vector3d tOppHomogA( tOppA.X(),  tOppA.Y(),  1 );
	const Vector3d tThisHomogA( tThisA.X(),  tThisA.Y(),  1 );
	Vector3d tOppHomogB( tOppB.X(),  tOppB.Y(),  1 );
	const Vector3d tThisHomogB( tThisB.X(),  tThisB.Y(),  1 );



	if ( fabs( tOppHomogB.Dot( F * tThisHomogA) ) < 
		fabs( tOppHomogA.Dot( F * tThisHomogA) ) )
		swap( tOppHomogA, tOppHomogB );

	//this order ensures that the A's and B's correspond
	//note that they do *not* automattically correspond
	//cases in which cameras are very close seem to be the opposite
	//I cannot quite visualise this yet
	//13-Nov-2003 it seems to depend on whether the cameras are facing or if one is behind the other


	Vector3d lineDirection, perpLineThruPoint;


	const Vector3d lineA = F_prime * tOppHomogA;
	lineDirection = Vector3d( lineA.X(), lineA.Y(), 0 );

	//M3PRINT(F_prime);
	//P3DPRINT(lineA);
	//P3DPRINT(tOppHomogA);

	perpLineThruPoint =  lineDirection.Cross( tThisHomogA );
	Vector3d pA = lineA.Cross(perpLineThruPoint);
	pA = pA/pA.Z();

	//P2DPRINT(pA);


	const Vector3d lineB = F_prime * tOppHomogB;	
	lineDirection = Vector3d( lineB.X(), lineB.Y(), 0 );
	perpLineThruPoint =  lineDirection.Cross( tThisHomogB );
	Vector3d pB = lineB.Cross(perpLineThruPoint);
	pB = pB/pB.Z();

	//P2DPRINT(pB);



	resVec[0] = pA.X() - tThisA.X();
	resVec[1] = pA.Y() - tThisA.Y();
	resVec[2] = pB.X() - tThisB.X();
	resVec[3] = pB.Y() - tThisB.Y();

	//mexPrintf("%g %g %g %g\n", 
	//	pA.X() - tThisA.X(),
	//	pA.Y() - tThisA.Y(),
	//	pB.X() - tThisB.X(),
	//	pB.Y() - tThisB.Y());

}


Wm3::Vector2d SilhouetteView::OrthoConvHullVertex( const int ind ) const
{
	return orthoBoundary[ convHullIndVec[ind] ];
}


Wm3::Vector2d SilhouetteView::ConvHullVertex( const int ind ) const
{
	return boundary[ convHullIndVec[ind] ];
}


Wm3::Vector2d SilhouetteView::ConvHullNiVertex( const int ind ) const
{
	return niBoundary[ convHullIndVec[ind] ];
}

void SilhouetteView::GeneratePointIndFromAngLut( const int numAngles )
{

	const double PI = 4*atan(double(1.0)); //3.14159265358979310000;

	//if (pointIndFromModGradMap.empty() )


	pointIndFromAngLut.clear();
	pointIndFromAngLut.resize( numAngles );

	vector<double> theEdgeAngle( convHullIndVec.size() );

	for ( unsigned int v = 0; v < convHullIndVec.size(); v++ )
	{

		int nextVertex = v + 1;

		if ( v == convHullIndVec.size() - 1 ) nextVertex = 0;


		double theAngle = PI/2 + 
			atan( 
			( OrthoConvHullVertex(v).Y() - OrthoConvHullVertex(nextVertex).Y() ) / 
			(OrthoConvHullVertex(v).X() - OrthoConvHullVertex(nextVertex).X()  ) );




		theEdgeAngle[v] = ( theAngle / PI ) * numAngles;


		if ( int(theEdgeAngle[v]) == numAngles ) 
			theEdgeAngle[v] = 0;
		// map 180 degrees to zero degrees


	}


	for ( unsigned int v = 0; v < convHullIndVec.size(); v++ )
	{		
		int leavingEdge = v;
		int arrivingEdge = v - 1;     
		if ( v == 0 ) arrivingEdge = convHullIndVec.size() - 1;        


		if (  theEdgeAngle[arrivingEdge] > theEdgeAngle[leavingEdge]  )
		{

			for (int loop = int(theEdgeAngle[arrivingEdge]); 
				loop <= numAngles - 1; loop++)
			{			             
				pointIndFromAngLut[loop].push_back( v );
			}

			for (int loop = 0; loop <= int(theEdgeAngle[leavingEdge]); loop++)
			{			             
				pointIndFromAngLut[loop].push_back( v );
			}         

		}
		else
		{
			for (int loop = int(theEdgeAngle[arrivingEdge]); 
				loop <= int(theEdgeAngle[leavingEdge]); loop++)
			{			  				
				pointIndFromAngLut[loop].push_back( v );
			}

		}

	}		
}


/*//////////////
void SilhouetteView::JitterBoundary( const double halfwidth )
{	
for( unsigned int vLoop = 0; vLoop < boundary.size(); vLoop++ )	
{	
const double randX = 
halfwidth * ( double( rand()+1 ) / double(RAND_MAX+2) - 0.5 );
boundary[vLoop].X() += randX;

const double randY = 
halfwidth * ( double( rand()+1 ) / double(RAND_MAX+2) - 0.5 );


boundary[vLoop].Y() += randY;

}
}
/*//////////////


void SilhouetteView::RemoveCollinearConvexHullVertices( const double tol )
{

	bool doneCheckWithNoChanges = false;
	//keep checking until there are no changes in the convex hull
	//one sweep will usually work, but not if there are several collinear points
	//in a row


	while (!doneCheckWithNoChanges)
	{

		doneCheckWithNoChanges = true; //start as true but my change back to false

		vector<int> newConvHullIndVec;
		newConvHullIndVec.reserve(  convHullIndVec.size()  );

		for(unsigned int hullLoop = 0; hullLoop < convHullIndVec.size(); hullLoop++ )	
		{	

			const int nextInd = (hullLoop + 1) % convHullIndVec.size();
			const int prevInd = (hullLoop + convHullIndVec.size() - 1) % convHullIndVec.size();

			//only add a point if its arriving and leaving  vectors are different

			Vector2d arrivingVec = 
				boundary[convHullIndVec[hullLoop]] - boundary[convHullIndVec[prevInd]];

			arrivingVec /= arrivingVec.Length();

			Vector2d leavingVec = 
				boundary[convHullIndVec[nextInd]] - boundary[convHullIndVec[hullLoop]];

			leavingVec /= leavingVec.Length();


			const bool isConvex = 
				-leavingVec.Y() * arrivingVec.X()
				+leavingVec.X() * arrivingVec.Y() >= 0;
			//for some reason, the qhull routine sometimes seems to give slighly nonconvex output


			const double dotProduct = leavingVec.Dot( arrivingVec );

			if (( 1-dotProduct > tol ) && isConvex )
			{			
				newConvHullIndVec.push_back( convHullIndVec[hullLoop] );			
			}
			else
			{
				doneCheckWithNoChanges = false; 
			}


		}

		convHullIndVec = newConvHullIndVec;
	}
}


//#define DO_EBERLY_CONVEX_HULL

#ifndef USE_WYKOBI
//#ifdef DO_EBERLY_CONVEX_HULL
void SilhouetteView::FormConvexHull()
{
	ConvexHull2d chull( boundary.size(), &(boundary[0]) );
	chull.DoIncremental();	
	int* hullIndices = chull.GetIndices();	
	convHullIndVec.resize( chull.GetQuantity() );
	for( unsigned int hullLoop = 0; hullLoop < chull.GetQuantity(); hullLoop++ )	
	{	
		convHullIndVec[hullLoop] = hullIndices[hullLoop];			
	}

	reverse( convHullIndVec.begin(),convHullIndVec.end() );

	RemoveCollinearConvexHullVertices( 1e-7 );

	//Qhull2DWrapper( boundary, convHullIndVec );

	//chPointVec.resize( hullIndVec.size() );
	//for( unsigned int hullLoop = 0; hullLoop < chPointVec.size(); hullLoop++ )	
	//{	
	//	chPointVec[hullLoop] = pointVec[hullIndVec[hullLoop]];					
	//}

}
#else
void SilhouetteView::FormConvexHull()
{


	const double signedArea = SignedArea();


	//DOUBLEPRINT(signedArea);

	//mexPrintf("%g\n",signedArea);
	if ( signedArea < 0 )
	{
		//mexPrintf("reversing\n");
		reverse( boundary.begin(), boundary.end() );
		//vector<Vector2d> tempVec = boundary;
		//for( unsigned int vLoop = 0; vLoop < boundary.size(); ++vLoop )
		//	boundary[vLoop] = tempVec[ boundary.size() - vLoop-1 ];
	}

	using namespace wykobi;
	polygon<double,2> poly; // = make_polygon<double>(make_circle<double>(1000.0,1000.0,100.0));

	for (unsigned int kk = 0; kk < boundary.size(); ++kk)
	{
		point2d<double> temp;
		temp.x = boundary[kk].X();
		temp.y =  boundary[kk].Y();
		temp.val = kk;
		poly.push_back( temp );				
	}		  		
	polygon<double,2> chull;

	wykobi::algorithm::convex_hull_melkman< 
		point2d<double> >(
		poly.begin(), poly.end(), std::back_inserter(chull));

	convHullIndVec.resize(  chull.size() );

	for( unsigned int hullLoop = 0; hullLoop < convHullIndVec.size(); hullLoop++ )	
	{	
		convHullIndVec[hullLoop] = chull[hullLoop].val;			
	}

	RemoveCollinearConvexHullVertices( 1e-7 );
}
#endif

void SilhouetteView::CopyConvexHullFromBoundary()
{
	convHullIndVec.resize( boundary.size() );

	for( unsigned int hullLoop = 0; hullLoop < boundary.size(); hullLoop++ )	
	{	
		convHullIndVec[hullLoop] = hullLoop;			
	}


}


double SilhouetteView::ModifiedGrad( const double xVal, const double yVal ) const
{
	const double grad = yVal/xVal;			
	double gradOrderVal;	
	if ( xVal > 0 )
	{
		if ( yVal > 0 )				
			gradOrderVal = 	grad+1; //hi: +inf, lo: +1				
		else			
			gradOrderVal = 1 / (1-grad); //hi: +1 lo: 0 			
	}
	else
	{
		if ( yVal > 0 )
			gradOrderVal = 	grad-1; //hi: -inf, lo: -1					
		else
			gradOrderVal = 	-1 / (grad+1); //hi: 0, lo: -1
	}

	return gradOrderVal;

}


void SilhouetteView::JitterBoundary( const double noiseLevel )
{
	for( unsigned int vLoop = 0; vLoop < boundary.size(); vLoop++ )
	{
		//add some jitter

		const double jitterX =  
			noiseLevel * (double( rand()+1 )/double(RAND_MAX+2)-0.5);

		const double jitterY =  
			noiseLevel * (double( rand()+1 )/double(RAND_MAX+2)-0.5);

		//DOUBLEPRINT(jitterX);
		//DOUBLEPRINT(jitterY);

		boundary[vLoop].X() += jitterX;
		boundary[vLoop].Y() += jitterY;


	}
}

//
//double SilhouetteView::ScaleOfInscribedConvProj( 
//	const std::vector<Wm3::Vector3d> pointVec, 
//	const Wm3::Vector3d centrePoint ) const
//{
//	vector<Vector2d> projPointVec( pointVec.size()  );
//	for (unsigned int pLoop=0; pLoop < pointVec.size(); ++pLoop)
//		camera.WorldToImage(pointVec[pLoop], projPointVec[pLoop]);
//
//	Vector2d projCentrePoint;
//	camera.WorldToImage(centrePoint, projCentrePoint);
//
//	if ( !Contains(projCentrePoint) )
//		return 0; //is the centre point falls outside the silhouette it has zero scale
//
//
//	vector<int> indVec;
//	Qhull2DWrapper( projPointVec, indVec);
//
//	/*mexPrintf("projPoly = [");
//	for (unsigned int indLoop=0; indLoop < indVec.size(); ++indLoop)
//	mexPrintf("[%g; %g] ", 
//	projPointVec[indVec[indLoop]].X(),
//	projPointVec[indVec[indLoop]].Y()  );
//	mexPrintf("];figure, ShowBoundary(projPoly); \n");*/
//
//	//double soFar = 0;
//	//for (unsigned int indLoop=1; indLoop <= indVec.size(); ++indLoop)
//	//{
//	//	soFar += 
//	//		projPointVec[indVec[indLoop % indVec.size()]].X() *
//	//		( projPointVec[indVec[(indLoop+1) % indVec.size()]].Y() - 
//	//		projPointVec[indVec[indLoop-1]].Y() );
//	//}
//
//	//DOUBLEPRINT(soFar);
//
//	/*	double soFar = 0;
//	for( unsigned int vLoop = 1; vLoop <= boundary.size(); vLoop++ )
//	{
//	soFar += 
//	boundary[vLoop % boundary.size()].X() *
//	( boundary[(vLoop+1) % boundary.size()].Y() - boundary[vLoop-1].Y() );
//	}
//	return soFar/2;*/
//
//	//mexPrintf("boundary = [");
//	//for (unsigned int pLoop=0; pLoop < boundary.size(); ++pLoop)
//	//	mexPrintf("[%g; %g] ", 
//	//	boundary[pLoop].X(),
//	//	boundary[pLoop].Y()  );
//	//mexPrintf("];hold on, ShowBoundary(boundary, 'type', 2); \n");
//
//	//need to check all obj vertices against all sil edges and
//	//all sil vertices against all obj edges, then choose the minimum
//
//
//	set<double> silToObjDistRatioSet;
//
//	//for (unsigned int pLoop=0; pLoop < 1; ++pLoop)
//
//	for (unsigned int pLoop=0; pLoop < boundary.size(); ++pLoop)
//	{
//		//INTEGERPRINT(pLoop);
//
//		const double distToSil = 
//			(boundary[pLoop] - projCentrePoint).Length();
//
//
//		const Vector2d unitVecToVertex = 
//			(boundary[pLoop] - projCentrePoint) / distToSil;
//
//
//		for (unsigned int indLoop=0; indLoop < indVec.size(); ++indLoop)
//		{
//			const int edgeEndInd = (indLoop + 1) % indVec.size();
//
//			if ( Sil::Kross(
//				boundary[pLoop]- projCentrePoint, 
//				projPointVec[indVec[indLoop]] - projCentrePoint) * 
//				Sil::Kross(boundary[pLoop]- projCentrePoint, 
//				projPointVec[indVec[edgeEndInd]]- projCentrePoint) <  0 )
//			{
//				const Vector2d edgeUnitNorm = 
//					(projPointVec[indVec[indLoop]] - 
//					projPointVec[indVec[edgeEndInd]]).UnitPerp();
//
//
//				//DOUBLEPRINT(edgeUnitNorm.Dot(unitVecToVertex));
//
//				if ( edgeUnitNorm.Dot(unitVecToVertex) < 0  )
//				{
//					//mexPrintf("%%no intersection\n");
//				}
//				else
//				{
//					//we have an intersection
//
//					/*	mexPrintf("%%intersection\n");
//
//					mexPrintf("edgeUnitNorm = [%g; %g]; ", 
//					edgeUnitNorm.X(), edgeUnitNorm.Y() );
//
//
//					mexPrintf("sil = [[%g; %g], [%g; %g]]; ", 
//					projCentrePoint.X(), projCentrePoint.Y(),
//					boundary[pLoop].X(), boundary[pLoop].Y()  );
//					mexPrintf("theEdge = [[%g; %g], [%g; %g]]; ", 
//					projPointVec[indVec[indLoop]].X(), projPointVec[indVec[indLoop]].Y(),
//					projPointVec[indVec[edgeEndInd]].X(), projPointVec[indVec[edgeEndInd]].Y()		);
//					mexPrintf("hold on, plot2D(sil, 'om-'), plot2D(sil(:,1), 'pb');plot2D(theEdge, '-r');plot2D(theEdge(:,1), 'sr');");
//					mexPrintf("hold on, plot2D([sil(:,1), sil(:,1)+45*edgeUnitNorm], 'b-');\n");*/
//
//
//
//
//
//					const double perpDistToEdge = 
//						edgeUnitNorm.Dot( projPointVec[indVec[edgeEndInd]] - projCentrePoint);
//					//dot with any edge point
//
//
//					const double distInDirToEdge = 
//						perpDistToEdge / unitVecToVertex.Dot(edgeUnitNorm);
//					//divide by cos theta
//
//					//DOUBLEPRINT(perpDistToEdge);
//					//P2DPRINT(unitVecToVertex);
//					//P2DPRINT(edgeUnitNorm);
//
//					//DOUBLEPRINT(distToSil);
//					//DOUBLEPRINT(distInDirToEdge);
//					silToObjDistRatioSet.insert( distToSil/distInDirToEdge );
//
//				}
//			}
//		}
//	}
//
//
//
//	//////
//	//now we check all the obj vertices against the silhouette edges
//	for (unsigned int indLoop=0; indLoop < indVec.size(); ++indLoop)
//	{
//		const double distToVertex = (projPointVec[indVec[indLoop]] - projCentrePoint).Length();
//		const Vector2d unitVecToVertex = (projPointVec[indVec[indLoop]] - projCentrePoint) / distToVertex;
//
//		for (unsigned int pLoop=0; pLoop < boundary.size(); ++pLoop)
//		{
//			const int edgeEndInd = (pLoop + 1) % boundary.size();
//
//			if ( Sil::Kross(
//				projPointVec[indVec[indLoop]] - projCentrePoint, 
//				boundary[pLoop] - projCentrePoint) * 
//				Sil::Kross(projPointVec[indVec[indLoop]] - projCentrePoint, 
//				boundary[edgeEndInd]- projCentrePoint) <  0 )
//			{
//				const Vector2d edgeUnitNorm = (boundary[pLoop] - boundary[edgeEndInd]).UnitPerp();
//				if ( edgeUnitNorm.Dot(unitVecToVertex) < 0  )
//				{				
//					const double perpDistToEdge = edgeUnitNorm.Dot( boundary[edgeEndInd] - projCentrePoint);			
//					const double distInDirToEdge = perpDistToEdge / unitVecToVertex.Dot(edgeUnitNorm);
//					silToObjDistRatioSet.insert( distInDirToEdge/distToVertex );
//
//				}
//			}
//		}
//	}
//
//
//	return (*silToObjDistRatioSet.begin());
//}
//
//
//bool SilhouetteView::Contains( const Wm3::Vector2d & point ) const
//{
//	PointInPolygon2d pntInPoly( boundary.size(), &(boundary[0]) );
//	return pntInPoly.Contains( point );
//}
//


void SilhouetteView::SetBoundingRectangle()
{
	boundRect[0] = boundary[0]; //starting values; changed below
	boundRect[1] = boundary[0]; //starting values; changed below

	for( int hullLoop = 0; hullLoop < NumConvexVertices(); hullLoop++ )
	{
		Vector2d point = boundary[ convHullIndVec[hullLoop] ];

		if (  point.X() < boundRect[0].X()  )
			boundRect[0].X() = point.X();

		if (  point.Y() < boundRect[0].Y()  )
			boundRect[0].Y() = point.Y();

		if (  point.X() > boundRect[1].X()  )
			boundRect[1].X() = point.X();

		if (  point.Y() > boundRect[1].Y()  )
			boundRect[1].Y() = point.Y();

	}

}

void SilhouetteView::SetBlobView( const mxArray* matlabVar )
{
	const string cameraFieldStr = "camera";
	const string boundaryFieldStr = "boundary";

	MSASSERT( mxGetFieldNumber( matlabVar, boundaryFieldStr.c_str()) >= 0  );
	MSASSERT( mxGetFieldNumber( matlabVar, cameraFieldStr.c_str()) >= 0 );

	const mxArray* matlabBoundary =	
		mxGetField( matlabVar, 0, boundaryFieldStr.c_str() );

	const int numVertices = mxGetN(matlabBoundary);

	boundary.resize(numVertices);
	for( int vLoop = 0; vLoop < numVertices; vLoop++ )					
		boundary[vLoop] = Vector2d(
		mxGetPr(matlabBoundary)[vLoop*2+0],
		mxGetPr(matlabBoundary)[vLoop*2+1] );


	SetUp(); //form convex hull, build map etc.
	SetBoundingRectangle();



	const mxArray* matlabCamera =				
		mxGetField( matlabVar, 0, cameraFieldStr.c_str() );

	camera.SetFromMatlabVar( matlabCamera, 0 );


}


void SilhouetteView::SetBlobView(
								 const std::vector<Wm3::Vector2d> & boundaryVal,
								 const VcalCamera & cameraVal, const Wm3::Vector2d & offset )
{
	boundary = boundaryVal;

	for( int vLoop = 0; vLoop < boundary.size(); vLoop++ )	
		boundary[vLoop] += offset;

	camera = cameraVal;

	SetUp(); //form convex hull, build map etc.
	SetBoundingRectangle();

}

bool SilhouetteView::DoesTouchBorder( const bool doMatlabImageOrigin) const
{
	double minX = 0;
	double minY = 0;
	double maxX = camera.Width()-1;
	double maxY = camera.Height()-1;

	/*DOUBLEPRINT(minX);
	DOUBLEPRINT(maxX);
	DOUBLEPRINT(minY);
	DOUBLEPRINT(maxY);*/
			
	if (doMatlabImageOrigin)
	{
		minX+=1;
		maxX+=1;
		minY+=1;
		maxY+=1;		
	}

//	INTEGERPRINT(convHullIndVec.size());
	for( unsigned int hullLoop = 0; hullLoop < convHullIndVec.size(); hullLoop++ )	
	{	
//		P2DPRINT(  boundary[ convHullIndVec[hullLoop] ]  );
		
		if	(boundary[ convHullIndVec[hullLoop] ].X() < minX) return true;
		if	(boundary[ convHullIndVec[hullLoop] ].X() > maxX) return true;		
		if	(boundary[ convHullIndVec[hullLoop] ].Y() < minY) return true;
		if	(boundary[ convHullIndVec[hullLoop] ].Y() > maxY) return true;				
	}

	return false;
}