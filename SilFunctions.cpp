#include<iostream>
#include<limits>
#include<string>
#include<sstream>
#include<cmath>
#include "SilFunctions.h"
#include "Wm3Math.h"

#define DOUBLEPRINT(str) mexPrintf("%% %s: %g\n", #str, str );
#define BOOLPRINT(str) mexPrintf("%% %s: %s\n", #str, str?"true":"false" );
#define INTEGERPRINT(str) mexPrintf("%% %s: %d\n", #str, str );
#define P2DPRINT(str) mexPrintf(" %s= [%g; %g]\n", #str, str.X(), str.Y() );
#define P3DPRINT(str) mexPrintf("%% %s: [%g; %g; %g]\n", #str, str.X(), str.Y() , str.Z() );
#define M4PRINT(str) mexPrintf("%% %s= [[%g,%g,%g,%g];[%g,%g,%g,%g];[%g,%g,%g,%g];[%g,%g,%g,%g]]\n", #str, str(0,0), str(0,1), str(0,2), str(0,3), str(1,0), str(1,1), str(1,2), str(1,3), str(2,0), str(2,1), str(2,2), str(2,3), str(3,0), str(3,1), str(3,2), str(3,3) );
#define M3PRINT(str) mexPrintf("%% %s= [[%g,%g,%g];[%g,%g,%g];[%g,%g,%g]]\n", #str, str(0,0), str(0,1), str(0,2), str(1,0), str(1,1), str(1,2), str(2,0), str(2,1), str(2,2) );



using namespace std;
using namespace Wm3;



double Sil::Kross( const Wm3::Vector2d & a, const Wm3::Vector2d & b )
{
	// returns Cross((x,y,0),(V.x,V.y,0)) = x*V.y - y*V.x
	return a.X() * b.Y() - a.Y() * b.X();
}

bool  Sil::IsBehind( const Wm3::Matrix4d & pose, 
					const Wm3::Matrix4d &  otherPose  ) 
{	
	const Vector4d camCentre( 0, 0, 0, 1);

	const Vector4d thisInOther = 
		otherPose * Sil::PoseInvert(pose) * camCentre;

	return   thisInOther.Z() < 0;

}


Wm3::Matrix4d Sil::PoseInvert( const Wm3::Matrix4d & pose ) 
{
	//Invert 4x4 rigid body transform matrices
	//matrix is assumed to represent a rigid body transform
	Matrix4d dest;	
	dest(0,0) = pose(0,0);
	dest(0,1) = pose(1,0);
	dest(0,2)  = pose(2,0);
	dest(0,3) = 
		-pose(0,0) * pose(0,3)
		-pose(1,0) * pose(1,3)
		-pose(2,0) * pose(2,3);		
	dest(1,0)  = pose(0,1);
	dest(1,1)  = pose(1,1);
	dest(1,2)  = pose(2,1);
	dest(1,3)=
		-pose(0,1) * pose(0,3)
		-pose(1,1) * pose(1,3)
		-pose(2,1) * pose(2,3);		
	dest(2,0)  = pose(0,2);
	dest(2,1)  = pose(1,2);
	dest(2,2)  = pose(2,2);
	dest(2,3)=
		-pose(0,2) * pose(0,3)
		-pose(1,2) * pose(1,3)
		-pose(2,2) * pose(2,3);		
	dest(3,0)=0.0;
	dest(3,1)=0.0;
	dest(3,2)=0.0;
	dest(3,3)=1.0;
	return dest;
}


void Sil::PrintPoly(  const std::vector<Wm3::Vector2d> & poly )
{
	mexPrintf("poly = [");
	for(unsigned int jj = 0; jj < poly.size(); ++jj )
		mexPrintf("[%g;%g] ", poly[jj].X(),  poly[jj].Y() );
	mexPrintf("];");
}

void Sil::RadialSamples( const std::vector<Wm3::Vector2d> & poly, 
						const int numSamples, 
						std::vector<double> & radialDistVec )
{

	radialDistVec.resize( numSamples );
	//radialDistVec.assign( numSamples,  numeric_limits<double>::infinity() );
	radialDistVec.assign( numSamples,  double(455.5) );

	//for (unsigned int rr=0; rr < radialDistVec.size(); ++rr)
		///mexPrintf("%g\n ", radialDistVec[rr] );
	
	

	const double angleStepSize = Mathd::TWO_PI / numSamples;

	DOUBLEPRINT( angleStepSize );

	vector<double> angleAtVertexVec( poly.size() );

	for (unsigned int pLoop=0; pLoop < angleAtVertexVec.size(); ++pLoop)
	{

		P2DPRINT( poly[pLoop] );

		angleAtVertexVec[pLoop] = Mathd::PI + atan2( 
			poly[pLoop].Y(), poly[pLoop].X() );

		DOUBLEPRINT( angleAtVertexVec[pLoop] );

	}

	for (unsigned int pLoop=0; pLoop < angleAtVertexVec.size(); ++pLoop)
	{
			
		//INTEGERPRINT( pLoop );

		const unsigned int nextInd = (pLoop+1) % angleAtVertexVec.size();

		 unsigned int indStart =  
			(unsigned int)( ceil( angleAtVertexVec[pLoop] / angleStepSize ) );

		 unsigned int indEnd =  
			(unsigned int)( floor( angleAtVertexVec[nextInd] / angleStepSize ) );

		if (   Kross( poly[pLoop], poly[nextInd] ) < 0   )
			swap(indStart, indEnd);


		//INTEGERPRINT( indStart );
		//DOUBLEPRINT( angleAtVertexVec[pLoop] );

		//INTEGERPRINT( indEnd );
		//DOUBLEPRINT( angleAtVertexVec[nextInd] );


		bool doNewline = false;
		for( unsigned int jj = indStart; 
			jj != (indEnd+1) % numSamples;
			jj = (jj+1)%numSamples )
		{

			doNewline = true;		
			mexPrintf("%d\n", jj);
			const double theAngle = double(jj) * angleStepSize;

			DOUBLEPRINT( theAngle * 180/3.1415926 );

			const Vector2d unitDir( 
				cos(theAngle - Mathd::PI ), sin(theAngle - Mathd::PI ) );

			const Vector2d edgeUnitNorm = 
				( poly[pLoop] - poly[nextInd] ).UnitPerp();

			const double perpDistToLine = 
				edgeUnitNorm.Dot( poly[pLoop] );

			//Vector2d unitVecToVertex = poly[pLoop];
			//unitVecToVertex.Normalize();

			const double distInDirToEdge = 
				fabs( perpDistToLine / unitDir.Dot(edgeUnitNorm) );

			if ( radialDistVec[jj] > distInDirToEdge )
				radialDistVec[jj] = distInDirToEdge;

		}
		if (doNewline)
		mexPrintf("\n");

	}


	mexPrintf("poly = [");
	for (unsigned int pLoop=0; pLoop < poly.size(); ++pLoop)
		mexPrintf("[%g;%g] ", poly[pLoop].X(), poly[pLoop].Y() );
	mexPrintf("];");

	mexPrintf("sampVec = [");
	for (unsigned int rr=0; rr < radialDistVec.size(); ++rr)
	{		
		const Vector2d samp = 
			radialDistVec[rr] * Vector2d( 
			cos(double(rr) * angleStepSize - Mathd::PI), 
			sin(double(rr) * angleStepSize - Mathd::PI) );
		mexPrintf("[%g;%g] ", samp.X(), samp.Y() );
	}
	mexPrintf("];");


	mexPrintf("figure, hold on, ShowPoly(poly), plot2D(sampVec, '.'),plot2D(poly(:,1), 'o'),plot2D(poly(:,end), 'v'), RadialPlot(sampVec, 1.333);\n");



}





//double Sil::EtErr( const std::vector< std::vector<SilhouetteView> > & silVecVec, 
//			 const int camA, const int blobA,
//			 const int camB, const int blobB )
//{
//
//	double resVec[4];
//	Matrix4d RbtAdjust( Matrix4d::IDENTITY );
//	silVecVec[camA][blobA].EtResiduals(
//		silVecVec[camB][blobB], 
//		RbtAdjust, resVec );
//	const double etErr = sqrt( 
//		(resVec[0]*resVec[0] + resVec[1]*resVec[1] + 
//		resVec[2]*resVec[2] + resVec[3]*resVec[3])/4 );
//
//	return etErr;
//
//}
//
//double Sil::EtErr( const std::vector< std::vector<SilhouetteView> > & silVecVec, 
//			 const int camA, const int blobA,
//			 const int camB, const int blobB, 
//			 const int camC, const int blobC )
//{
//
//	const double etErrAB = EtErr( silVecVec, camA, blobA,camB,blobB);
//	const double etErrBC = EtErr( silVecVec, camB,blobB, camC, blobC);
//	const double etErrCA = EtErr( silVecVec, camC, blobC, camA, blobA);
//	return sqrt( (etErrAB*etErrAB+etErrBC*etErrBC+etErrCA*etErrCA) / 3 );
//}




