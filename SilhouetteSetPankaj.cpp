#include <iostream>
#include <string>
#include <algorithm>
//#include "Wm3ContConvexHull2.h"
#include "Wm3Matrix4.h"
#include "Wm3Matrix3.h"
#include "Wm3Vector4.h"
#include "Wm3Vector3.h"
#include "Wm3Vector2.h"
#include "Wm3Quaternion.h"
//#include "GslLevenbergMarquardt.h"
//#include "TriangulateErrFn.h"
//#include "EtOrthoErrFn.h"
//#include "EtPerspErrFn.h"
//#include "EtOrthoDirtRobustErrFn.h"
//#include "EtScaleOrthoErrFn.h"
//#include "RigidTransform.h"
#include "SilhouetteSet.h"



#define for if(0); else for // Required for Windows for-loop scoping issues.
#define DOUBLEPRINT(str) mexPrintf("%% %s: %g\n", #str, str );
#define BOOLPRINT(str) mexPrintf("%% %s: %s\n", #str, str?"true":"false" );
#define INTEGERPRINT(str) mexPrintf("%% %s: %d\n", #str, str );
#define P2DPRINT(str) mexPrintf(" %s= [%g; %g]\n", #str, str.X(), str.Y() );
#define P3DPRINT(str) mexPrintf("%% %s: [%g; %g; %g]\n", #str, str.X(), str.Y() , str.Z() );
#define M4PRINT(str) mexPrintf("%% %s= [[%g,%g,%g,%g];[%g,%g,%g,%g];[%g,%g,%g,%g];[%g,%g,%g,%g]]\n", #str, str(0,0), str(0,1), str(0,2), str(0,3), str(1,0), str(1,1), str(1,2), str(1,3), str(2,0), str(2,1), str(2,2), str(2,3), str(3,0), str(3,1), str(3,2), str(3,3) );
#define M3PRINT(str) mexPrintf("%% %s= [[%g,%g,%g];[%g,%g,%g];[%g,%g,%g]]\n", #str, str(0,0), str(0,1), str(0,2), str(1,0), str(1,1), str(1,2), str(2,0), str(2,1), str(2,2) );


using namespace Wm3;
using namespace std;

void  SilhouetteSet::SetCameraPose( const int camInd, const Wm3::Matrix4d & pose )
{
	viewVec[camInd].camera.Pose() = pose;
}

void  SilhouetteSet::GetCameraPose( const int camInd,  Wm3::Matrix4d & pose ) const
{
	pose = viewVec[camInd].camera.Pose();
}

void SilhouetteSet::GetCamera( const int camInd, VcalCamera &cam ) const
{
	cam = viewVec[camInd].camera;
}

void SilhouetteSet::SetCamera( const int camInd, const VcalCamera &cam ) 
{
	 viewVec[camInd].camera = cam;
}


void SilhouetteSet::SetCameraEfl( const int camInd, double eflVal )
{
	viewVec[camInd].camera.Efl() = eflVal;
}

void SilhouetteSet::SetCameraU0( const int camInd, double u0Val )
{
	viewVec[camInd].camera.U0() = u0Val;
}

void SilhouetteSet::SetCameraV0( const int camInd, double v0Val )
{
	viewVec[camInd].camera.V0() = v0Val;
}

double SilhouetteSet::GetCameraEfl( const int camInd ) const
{
	return viewVec[camInd].camera.Efl();
}

double SilhouetteSet::GetCameraU0( const int camInd ) const
{
	return viewVec[camInd].camera.U0();
}

double SilhouetteSet::GetCameraV0( const int camInd ) const
{
	return viewVec[camInd].camera.V0();
}


//void SilhouetteSet::LmCentrePoint( const double tol, 
//								  const long maxIterations, 
//								  Wm3::Vector3d & centrePoint ) const
//{
//	ApproxCentrePoint(centrePoint); //initial estimate
//
//
//	vector<Vector2d> imgCentroidVec( NumViews() );
//	vector<VcalCamera> camVec( NumViews() );
//
//	for( int viewLoop = 0; viewLoop < NumViews(); viewLoop++ )
//	{
//		viewVec[viewLoop].ConvexSilhouetteCentroid( imgCentroidVec[viewLoop] );
//		camVec[viewLoop] = viewVec[viewLoop].camera;
//	}
//
//
//	TriangulateErrFn ef( &imgCentroidVec, &camVec );
//
//	GslLevenbergMarquardt lm( &ef );	
//	lm.SetTolF( tol );	
//	lm.SetMaxIter( maxIterations );
//
//
//	vector<double> initVec(3);
//	initVec[0] = centrePoint.X();
//	initVec[1] = centrePoint.Y();
//	initVec[2] = centrePoint.Z();
//
//	vector<double> optVec(3);
//	lm.Minimize( initVec, optVec );
//
//
//	centrePoint.X() = optVec[0];
//	centrePoint.Y() = optVec[1];
//	centrePoint.Z() = optVec[2];
//
//
//	//mexPrintf("%f pixels\n", ef.RmsErr(initVec) );
//	//mexPrintf("%f pixels\n", ef.RmsErr(optVec) );
//
//
//}
//double SilhouetteSet::MeanConvexOrthoArea(  ) const
//{
//
//	double tot = 0;
//	for( int viewLoop = 0; viewLoop < NumViews(); viewLoop++ )
//	{
//		tot+= fabs( viewVec[viewLoop].SignedConvexOrthoArea() );
//	}
//
//	//DOUBLEPRINT(tot/NumViews());
//	//DOUBLEPRINT(tot);
//
//	return tot/NumViews();
//}
//
//void SilhouetteSet::ApproxCentrePoint( Wm3::Vector3d & centrePoint ) const
//{
//	vector<Vector2d> imgCentroidVec( NumViews() );
//	vector<Vector2d> niCentroidVec( NumViews() );
//
//	for( int viewLoop = 0; viewLoop < NumViews(); viewLoop++ )
//	{
//		viewVec[viewLoop].ConvexSilhouetteCentroid( imgCentroidVec[viewLoop] );
//
//		niCentroidVec[viewLoop] = 
//			viewVec[viewLoop].camera.ImageToNormalisedImage(imgCentroidVec[viewLoop]);
//	}
//
//
//	vector<double> bb( 2*NumViews() );
//
//
//	GMatrix<double> AA( 2*NumViews(), 3 );
//
//	for( int viewLoop = 0; viewLoop < NumViews(); viewLoop++ )
//	{
//
//		Matrix3d R;		
//		viewVec[viewLoop].camera.CreateR(R);
//		Vector3d t;		
//		viewVec[viewLoop].camera.Create_t(t);
//
//		Vector3d rowVec;
//
//		rowVec = R.GetRow(0) - niCentroidVec[viewLoop].X() * R.GetRow(2);
//
//		for( int cc=0;cc<3;cc++)
//			AA(viewLoop * 2 + 0, cc) = rowVec[cc];
//
//		rowVec = R.GetRow(1) - niCentroidVec[viewLoop].Y() * R.GetRow(2);
//
//		for( int cc=0;cc<3;cc++)
//			AA(viewLoop * 2 + 1, cc) = rowVec[cc];
//
//
//		//Xi{cam}(1, thePoint) * KRt{cam}(3,4) - KRt{cam}(1,4)
//
//		bb[viewLoop * 2 + 0] = niCentroidVec[viewLoop].X() * t.Z() - t.X();
//		bb[viewLoop * 2 + 1] = niCentroidVec[viewLoop].Y() * t.Z() - t.Y();
//
//	}
//
//	//A( (cam-1) * 2 + 1, : ) = ( KRt{cam}(1,1:3) - Xi{cam}(1, thePoint) * KRt{cam}(3,1:3) );
//	//b( (cam-1) * 2 + 1, 1 ) = Xi{cam}(1, thePoint) * KRt{cam}(3,4) - KRt{cam}(1,4);        
//	//A( (cam-1) * 2 + 2, : ) = ( KRt{cam}(2,1:3) - Xi{cam}(2, thePoint) * KRt{cam}(3,1:3) );
//	//b( (cam-1) * 2 + 2, 1 ) = Xi{cam}(2, thePoint) * KRt{cam}(3,4) - KRt{cam}(2,4);
//
//
//	/*
//	for (int rr = 0; rr < 2*NumViews() ; rr++)
//	{
//	mexPrintf("AA: %f %f %f\n", 
//	AA(rr,0), AA(rr,1), AA(rr,2) );
//	}
//
//	for (int rr = 0; rr < 2*NumViews() ; rr++)
//	{
//	mexPrintf("bb: %f\n", bb[rr] );
//	}
//
//	*/
//	LinearSystemd linsys;
//
//	if ( !linsys.Solve( AA, &(bb[0]), &(centrePoint[0]) ) )
//	{
//		throw string("Failed to solve linear system.");
//	}
//
//
//
//}
//
void SilhouetteSet::EtResiduals( const int viewThisInd, 
								const int viewOppInd, double * resVec ) const
{	
	viewVec[viewThisInd].EtResiduals( viewVec[viewOppInd], Matrix4d::IDENTITY, resVec );
}


//void SilhouetteSet::SetMomentAlignFromQuatTran( double * vec )
//{
//	Quaterniond quat( vec[3], vec[0], vec[1], vec[2] ); //keep original matlab order
//	quat.Normalize();
//	//eberly happens to strore quaternions as w,x,y,z, but this can mostly be ignored
//	//only really need to watch out when using the Quaternion constructor which takes the 4 elements in a differnt oreder to matlab
//
//	Matrix3d R;
//	quat.ToRotationMatrix(R);
//
//
//	momentAlign = Matrix4d(
//		R(0,0), R(0,1), R(0,2), 0,
//		R(1,0), R(1,1), R(1,2), 0,
//		R(2,0), R(2,1), R(2,2), 0,
//		0,      0,      0,      1);
//
//}
//

//void SilhouetteSet::SetMomentAlign( double * data )
//{
//	for(int rr = 0; rr<4; rr++)
//		for(int cc = 0; cc<4; cc++)
//			momentAlign(rr,cc) = data[rr+4*cc];
//}

//double SilhouetteSet::MinimiseEtErrFromPoseScale( SilhouetteSet & silsetOther, 
//												 Wm3::Matrix3d orientationMatrix,				
//												 const int maxIterations ,
//												 const double tol,
//												 const int propReductionRange,
//												 const double propReductionThreshold )
//{
//	Matrix4d poseMatrix(	
//		orientationMatrix(0,0), orientationMatrix(0,1), orientationMatrix(0,2), 0,
//		orientationMatrix(1,0), orientationMatrix(1,1), orientationMatrix(1,2), 0,
//		orientationMatrix(2,0), orientationMatrix(2,1), orientationMatrix(2,2), 0,
//		0, 0, 0, 1);			
//	EtScaleOrthoErrFn ef( this, &silsetOther );
//	GslLevenbergMarquardt lm( &ef );	
//	lm.SetMaxIter( maxIterations );	
//	lm.SetPropRedThresh( propReductionThreshold );		
//	lm.SetPropRedRange( propReductionRange );					
//	vector<double> optVec( ef.NumParams()  );
//	vector<double> initVec( ef.NumParams()  );
//	const double mcaA = this->MeanConvexOrthoArea();	
//	const double mcaB = silsetOther.MeanConvexOrthoArea();		
//	const RigidTransform rt( (this->MomentAlign()).Inverse() * poseMatrix * silsetOther.MomentAlign() );
//	rt.Create( &(initVec[0]) );
//	initVec[7] = sqrt( mcaA / mcaB);
//	lm.Minimize( initVec, optVec );					
//	vector<double> rmsVec;
//	lm.GetRmsRec( rmsVec );			
//	RigidTransform rtOpt( &(optVec[0]) );
//	Matrix4d RbtAdjust;
//	rtOpt.Create( RbtAdjust );	
//	RbtAdjust *= optVec[8-1];
//	RbtAdjust(4-1,4-1) = 1;
//	vector<double> resVec( 8*this->NumViews() * silsetOther.NumViews() );	
//	vector<double> epiInsideVec( resVec.size() );	
//	int count = 0;
//	for (int viewLoopA = 0; viewLoopA < this->NumViews(); viewLoopA++ )
//		for (int viewLoopB = 0; viewLoopB < silsetOther.NumViews(); viewLoopB++ )
//		{
//			double resData[8];
//			this->View(viewLoopA).EtResiduals( silsetOther.View(viewLoopB), RbtAdjust, resData );
//			silsetOther.View(viewLoopB).EtResiduals( this->View(viewLoopA), RbtAdjust.Inverse(), &(resData[4]) );				
//			for( int fourLoop = 0; fourLoop < 4; fourLoop++)			
//			{
//				epiInsideVec[count+fourLoop + 0] =
//					this->View(viewLoopA).IsEpipoleInBoundRect( silsetOther.View(viewLoopB), RbtAdjust ) ? 1 : 0;
//				epiInsideVec[count+fourLoop + 4] =
//					silsetOther.View(viewLoopB).IsEpipoleInBoundRect( 
//					this->View(viewLoopA), RbtAdjust.Inverse() ) ? 1 : 0;
//			}
//			for( int eightLoop = 0; eightLoop < 8; eightLoop++)					
//				resVec[count+eightLoop] = resData[eightLoop];															
//			count += 8;											
//		}					
//		//now we need to replace the epipole inside cases with values that will
//		//leave the rms  as if they were removed
//		//doing this instead of removing them keeps the vector length
//		//the same so Levenberg-Marquardt doesn't break
//		double sumSqSoFar = 0;
//		int numValidSoFar = 0;
//		for( int resLoop = 0; resLoop < resVec.size(); resLoop++ )				
//			if ( 0 == epiInsideVec[resLoop] )
//			{
//				numValidSoFar++;
//				sumSqSoFar = sumSqSoFar + resVec[resLoop]*resVec[resLoop];
//			}
//
//			const double etPixelErr = sqrt(sumSqSoFar/numValidSoFar);
//			return etPixelErr;
//}
//
//double SilhouetteSet::MinimiseEtErrFromPose( SilhouetteSet & silsetOther, 
//											Wm3::Matrix3d orientationMatrix,							
//											const int maxIterations ,
//											const double tol ,
//											const int propRedRange ,
//											const double propRedThresh )
//{
//	EtOrthoErrFn efOrtho( this,  &silsetOther );
//	EtPerspErrFn efPersp( this,  &silsetOther );
//	Matrix4d poseMatrix(	
//		orientationMatrix(0,0), orientationMatrix(0,1), orientationMatrix(0,2), 0,
//		orientationMatrix(1,0), orientationMatrix(1,1), orientationMatrix(1,2), 0,
//		orientationMatrix(2,0), orientationMatrix(2,1), orientationMatrix(2,2), 0,
//		0, 0, 0, 1);
//	const RigidTransform rt( momentAlign.Inverse() * poseMatrix * silsetOther.momentAlign );
//	vector<double> vecInit(7);
//	rt.Create( &(vecInit[0]) );
//	GslLevenbergMarquardt lm( &efOrtho, tol, maxIterations, propRedThresh, propRedRange  );	
//	vector<double> vecOpt( efOrtho.NumParams()  );			
//	lm.Minimize( vecInit, vecOpt );
//	return efPersp.RmsErr(vecOpt);		
//}
//
//bool SilhouetteSet::IsMatch( SilhouetteSet & silsetOther, 
//							Wm3::Matrix3d orientationMatrix,
//							const double etThresh, 							
//							const int maxIterations ,
//							const double tol ,
//							const int propRedRange ,
//							const double propRedThresh ,
//							const int numRandStartingPoints )
//{
//
//	const double etErr = MinimiseEtErrFromPose( silsetOther, orientationMatrix,							
//		maxIterations,tol, propRedRange, propRedThresh);
//
//	return ( etErr < etThresh ) ? true : false;
//
//}
//
//
//
//double SilhouetteSet::AlignFromMoments( SilhouetteSet & silsetOther, 
//									   Wm3::Matrix4d orientMat,
//									   const int maxIterations,
//									   const double tol,
//									   const int propRedRange,
//									   const double propRedThresh)
//{
//
//	EtOrthoErrFn efOrtho( this,  &silsetOther );
//	EtPerspErrFn efPersp( this,  &silsetOther );
//
//	const Matrix4d RbtAlign = 
//		momentAlign.Inverse() * 
//		orientMat * 
//		silsetOther.momentAlign;
//
//	const Matrix3d R(
//		RbtAlign(0,0), RbtAlign(0,1), RbtAlign(0,2), 
//		RbtAlign(1,0), RbtAlign(1,1), RbtAlign(1,2), 
//		RbtAlign(2,0), RbtAlign(2,1), RbtAlign(2,2) );
//
//	Quaterniond quat;
//	quat.FromRotationMatrix(R);
//
//	vector<double> vecInit(7);
//
//	vecInit[0] = quat.X();
//	vecInit[1] = quat.Y();
//	vecInit[2] = quat.Z();
//	vecInit[3] = quat.W(); //this is the original ordering used in Matlab -- we need to be consitent with this
//	vecInit[4] = RbtAlign(0,3);
//	vecInit[5] = RbtAlign(1,3);
//	vecInit[6] = RbtAlign(2,3);
//
//	GslLevenbergMarquardt lm( &efOrtho );	
//	lm.SetTolF( tol );	
//	lm.SetMaxIter( maxIterations );
//	lm.SetPropRedThresh(propRedThresh);
//	lm.SetPropRedRange(propRedRange);
//
//	vector<double> vecOpt( efOrtho.NumParams()  );			
//	lm.Minimize( vecInit, vecOpt );
//
//	return efPersp.RmsErr(vecOpt);	
//}
//
//
//bool SilhouetteSet::IsMatch( SilhouetteSet & silsetOther, 
//							std::vector<Wm3::Matrix4d> orientVec,
//							const double etThresh, 
//							int & exitType,
//							const int maxIterations,
//							const double tol,
//							const int propRedRange,
//							const double propRedThresh,
//							const int numRandStartingPoints )
//{
//	//exit as soon as a pose is found that provides an ET error less than the threshold
//
//	EtOrthoErrFn efOrtho( this,  &silsetOther );
//	EtPerspErrFn efPersp( this,  &silsetOther );
//
//	for( unsigned int oriLoop = 0; oriLoop < orientVec.size(); ++oriLoop)
//	{
//
//		const Matrix4d RbtAlign = 
//			momentAlign.Inverse() * 
//			orientVec[oriLoop] * 
//			silsetOther.momentAlign;
//
//		const Matrix3d R(
//			RbtAlign(0,0), RbtAlign(0,1), RbtAlign(0,2), 
//			RbtAlign(1,0), RbtAlign(1,1), RbtAlign(1,2), 
//			RbtAlign(2,0), RbtAlign(2,1), RbtAlign(2,2) );
//
//		Quaterniond quat;
//		quat.FromRotationMatrix(R);
//
//		vector<double> vecInit(7);
//
//		vecInit[0] = quat.X();
//		vecInit[1] = quat.Y();
//		vecInit[2] = quat.Z();
//		vecInit[3] = quat.W(); //this is the original ordering used in Matlab -- we need to be consitent with this
//		vecInit[4] = RbtAlign(0,3);
//		vecInit[5] = RbtAlign(1,3);
//		vecInit[6] = RbtAlign(2,3);
//
//
//		GslLevenbergMarquardt lm( &efOrtho );	
//		lm.SetTolF( tol );	
//		lm.SetMaxIter( maxIterations );
//		lm.SetPropRedThresh(propRedThresh);
//		lm.SetPropRedRange(propRedRange);
//
//		vector<double> vecOpt( efOrtho.NumParams()  );			
//		lm.Minimize( vecInit, vecOpt );
//
//
//		const double etErrPersp = efPersp.RmsErr(vecOpt);
//		//DOUBLEPRINT(etErrPersp);
//		if ( etErrPersp < etThresh )
//		{
//			exitType = oriLoop;
//			return true;
//		}
//
//
//	}
//
//
//
//	exitType = -1;
//	return false;
//}
//
//
//bool SilhouetteSet::IsMomentMatch( SilhouetteSet & silsetOther, 
//								  const double etThresh, 
//								  int & exitType,
//								  const int maxIterations,
//								  const double tol,
//								  const int propRedRange,
//								  const double propRedThresh,
//								  const int numRandStartingPoints )
//{
//	//exit as soon as a pose is found that provides an ET error less than the threshold
//
//
//	//for alignLoop = 1:4,
//	// vecMoment(:,alignLoop) = Matrix4x4ToQuatTran( ...
//	//   inv(  QuatTranToMatrix4x4( vecQuatTranArrayCell{1}(:,1) )  ) * ...
//	// QuatTranToMatrix4x4( vecQuatTranArrayCell{2}(:,alignLoop) ) );                                       
//
//	//end
//
//
//	//first we'll try the four moment alignments
//	//starting with 
//
//	vector<Matrix4d> alignMatrix(4);
//
//	//alignMatrix
//
//
//	//Matrix4d alignMatrix[4];
//
//	//alignMatrix.push_back( Matrix4d::IDENTITY );
//
//	alignMatrix[0] = Matrix4d::IDENTITY;
//	alignMatrix[0](0,0) = +1;
//	alignMatrix[0](1,1) = +1;
//	alignMatrix[0](2,2) = +1;
//	alignMatrix[1] = Matrix4d::IDENTITY;
//	alignMatrix[1](0,0) = -1;
//	alignMatrix[1](1,1) = -1;
//	alignMatrix[1](2,2) = +1;
//	alignMatrix[2] = Matrix4d::IDENTITY;
//	alignMatrix[2](0,0) = -1;
//	alignMatrix[2](1,1) = +1;
//	alignMatrix[2](2,2) = -1;
//	alignMatrix[3] = Matrix4d::IDENTITY;
//	alignMatrix[3](0,0) = +1;
//	alignMatrix[3](1,1) = -1;
//	alignMatrix[3](2,2) = -1;
//
//
//	return IsMatch( silsetOther, alignMatrix, etThresh, exitType, 
//		maxIterations, tol,propRedRange, propRedThresh, 
//		numRandStartingPoints );
//}
//
//
//
//bool SilhouetteSet::IsQuatMatch( SilhouetteSet & silsetOther, 
//								const int samplesPerAxis,
//								const double etThresh, 
//								int & exitType,
//								const int maxIterations,
//								const double tol,
//								const int propRedRange,
//								const double propRedThresh,
//								const int numRandStartingPoints )
//{
//	QuaternionGridSet quatGridSet(samplesPerAxis, true);
//
//
//	vector<Matrix4d> orientVec( quatGridSet.NumElements() );
//
//	for( int oriLoop = 0; oriLoop < quatGridSet.NumElements(); oriLoop++ )
//	{
//		orientVec[oriLoop] = Matrix4d::IDENTITY;
//
//		const Quaterniond quat = quatGridSet(oriLoop);
//
//		Matrix3d R;
//		quat.ToRotationMatrix(R);
//
//		for( int rr = 0; rr < 3; rr++ )
//			for( int cc = 0; cc < 3; cc++ )
//				orientVec[oriLoop](rr,cc) = R(rr,cc);
//
//	}
//
//	return IsMatch( silsetOther, orientVec, etThresh, exitType, 
//		maxIterations, tol,propRedRange, propRedThresh, 
//		numRandStartingPoints );
//}
//
//
//
//void SilhouetteSet::CreateOrthoBoundaries( const double lmTol, const long lmMfe )
//{
//
//
//	Vector3d centrePoint;
//
//	LmCentrePoint(lmTol, lmMfe, centrePoint);
//
//
//	for(unsigned int viewLoop = 0; viewLoop < viewVec.size(); viewLoop++)
//		viewVec[viewLoop].CreateOrthoBoundary( centrePoint );
//}

void SilhouetteSet::CreateLuts( const int numAngles )
{
	for(unsigned int viewLoop = 0; viewLoop < viewVec.size(); viewLoop++)
		viewVec[viewLoop].GeneratePointIndFromAngLut( numAngles );		
}




//void SilhouetteSet::ConstDepthRims(  std::vector<Wm3::Vector3d> & pointVec, 
//								   std::vector<int> & labelVec ) const
//{
//
//	Vector3d centrePoint;
//	ApproxCentrePoint(centrePoint);
//
//	//P3DPRINT(centrePoint);
//
//	pointVec.resize(0);
//	labelVec.resize(0);
//
//	for(unsigned int viewLoop = 0; viewLoop < viewVec.size(); viewLoop++)
//	{
//
//		vector<Vector3d> rimVec;
//		viewVec[viewLoop].CreateConstDepthConvRim( centrePoint, rimVec );
//
//		//INTEGERPRINT(rimVec.size());
//
//		if (0==viewLoop)
//		{
//			const int reserveSize = rimVec.size() * viewVec.size() * 2;
//			pointVec.reserve( reserveSize );
//			labelVec.reserve( reserveSize );
//		}
//
//		for( unsigned int vertexLoop = 0; vertexLoop < rimVec.size(); ++vertexLoop )
//		{
//			pointVec.push_back( rimVec[vertexLoop] );
//			labelVec.push_back( viewLoop );
//		}
//
//	}
//}
//
////
//void SilhouetteSet::VisualHullRayIntersection(
//	const Vector3d & rayCentre,
//	const Vector3d & rayDirection,
//	Vector3d & intersectionPoint )
//{
//	for(unsigned int viewLoop = 0; viewLoop < viewVec.size(); viewLoop++)
//	{
//
//
//	}
//}
//







double SilhouetteSet::AngleAtEpipole( const int viewA, const int viewB ) const
{

	Vector2d epipole;

	viewVec[viewA].camera.Epipole( viewVec[viewB].camera, Matrix4d::IDENTITY, 
		epipole	 );


	//	mexPrintf("(%g, %g)\n", epipole.X(),  epipole.Y() );

	return viewVec[viewA].AngleAtEpipole(epipole);



}

bool SilhouetteSet::IsEpipoleInBoundRect( const int viewA, const int viewB ) const
{
	// 26/10/2006 - Added by Mathew for checking whether camera centres are identical.
	Matrix4d PoseA = viewVec[viewA].camera.Pose();
	Matrix4d PoseB = viewVec[viewB].camera.Pose();
	PoseA = PoseA.Inverse();
	PoseB = PoseB.Inverse();
	Vector4d tranA = PoseA.GetColumn(3);
	Vector4d tranB = PoseB.GetColumn(3);    
	if (tranA == tranB) {
		return 1;
	}
	// End of addition.    

	Vector2d epipole;
	viewVec[viewA].camera.Epipole( viewVec[viewB].camera, Matrix4d::IDENTITY, 
		epipole  );


	return viewVec[viewA].IsEpipoleInBoundRect(epipole);

}

void SilhouetteSet::ConvBoundPoint( const int kInd, 
								   const int viewInd, Vector2d & point ) const
{
	point = viewVec[viewInd].boundary[ viewVec[viewInd].convHullIndVec[kInd] ];
}


int SilhouetteSet::ConvBoundNumPoints( const int viewInd ) const
{
	return  viewVec[viewInd].convHullIndVec.size();
}




SilhouetteSet::SilhouetteSet()
{
}



#ifdef MATLAB_MEX_FILE
SilhouetteSet::SilhouetteSet( const mxArray* matlabViewVec,
							 SpecialSilhouetteSetConstruction flagVal )
{

	//mexPrintf("vrevrevrevervrere\n");


	switch (flagVal)
	{
	case silhouetteSetWithSingleSilhouette:
		SetFromMatlabViewVec( matlabViewVec );
		break;

	case silhouetteSetWithUnorderedConvexVertices:
		SetFromMatlabConvexUnorderedViewVec( matlabViewVec );
		//CreateOrthoBoundaries();
		CreateLuts();
		break;

	case constructWithoutLut:		
		SetFromMatlabViewVec( matlabViewVec );
		//CreateOrthoBoundaries();
		

		break;

	case constructWithoutLutOrOrtho:
		SetFromMatlabViewVec( matlabViewVec );		
		break;


	default:
		string errMssg("missing boundary field in viewVec from Matlab.");
		throw errMssg;	
		break;
	}

	//JitterBoundaries( 0.0000001 );
	//required for 
}	
#endif



#ifdef MATLAB_MEX_FILE
SilhouetteSet::SilhouetteSet( const mxArray* matlabViewVec )
{
	//	mexPrintf("rweferf\n");
	SetFromMatlabViewVec( matlabViewVec );
	//mexPrintf("rweferf\n");
	//SetFromMatlabViewVec( matlabViewVec );

	//CreateOrthoBoundaries();
	//mexPrintf("rweferf\n");
	//CreateOrthoBoundaries();
}

// Use this constructor if data may contain silhouette sets with only one view.
SilhouetteSet::SilhouetteSet( const mxArray* matlabViewVec, int single )
{
	SetFromMatlabViewVec( matlabViewVec );
}

#endif



#ifdef MATLAB_MEX_FILE
void SilhouetteSet::SetFromMatlabConvexUnorderedViewVec( const mxArray* matlabViewVec )
{
	string cameraFieldStr = "camera";
	string boundaryFieldStr = "boundary";
	const int numViews = mxGetNumberOfElements( matlabViewVec );
	viewVec.resize(numViews);

	for(int viewLoop = 0; viewLoop < numViews; viewLoop++)
	{
		if  ( mxGetFieldNumber( matlabViewVec, boundaryFieldStr.c_str() ) < 0 ) 
		{
			string errMssg("missing boundary field in viewVec from Matlab.");
			throw errMssg;	
		}

		const mxArray* matlabBoundary =				
			mxGetField( matlabViewVec, viewLoop, boundaryFieldStr.c_str() );

		const int numVertices = mxGetN(matlabBoundary);
		viewVec[viewLoop].boundary.resize(numVertices);


		Vector2d meanPoint(0,0);
		for( int vLoop = 0; vLoop < numVertices; vLoop++ )
		{
			viewVec[viewLoop].boundary[vLoop] = Vector2d(
				mxGetPr(matlabBoundary)[vLoop*2+0],
				mxGetPr(matlabBoundary)[vLoop*2+1] );

			meanPoint += viewVec[viewLoop].boundary[vLoop];
		}
		meanPoint /= numVertices;


		//we need to order the points
		//they are require to be on a convexhull, but are unordered.

		vector< pair<double, Vector2d> > gradIndexVec( numVertices );

		for( int vLoop = 0; vLoop < numVertices; vLoop++ )
		{		
			const double xVal = 
				(viewVec[viewLoop].boundary[vLoop].X() - meanPoint.X());		
			const double yVal = 
				(viewVec[viewLoop].boundary[vLoop].Y() - meanPoint.Y());


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




			gradIndexVec[vLoop] = 
				make_pair( gradOrderVal, viewVec[viewLoop].boundary[vLoop] );
		}

		sort( gradIndexVec.begin(), gradIndexVec.end() );

		for( int vLoop = 0; vLoop < numVertices; vLoop++ )
		{	
			viewVec[viewLoop].boundary[vLoop] = 
				gradIndexVec[vLoop].second;

		}

		//viewVec[viewLoop].FormConvexHull();
		//
		viewVec[viewLoop].CopyConvexHullFromBoundary();

		viewVec[viewLoop].boundRect[0] = viewVec[viewLoop].boundary[0];
		viewVec[viewLoop].boundRect[1] = viewVec[viewLoop].boundary[0];


		for( int hullLoop = 0; 
			hullLoop < viewVec[viewLoop].NumConvexVertices(); hullLoop++ )
		{

			Vector2d point;
			ConvBoundPoint( hullLoop, viewLoop, point );

			if (  point.X() < viewVec[viewLoop].boundRect[0].X()  )
				viewVec[viewLoop].boundRect[0].X() = point.X();

			if (  point.Y() < viewVec[viewLoop].boundRect[0].Y()  )
				viewVec[viewLoop].boundRect[0].Y() = point.Y();

			if (  point.X() > viewVec[viewLoop].boundRect[1].X()  )
				viewVec[viewLoop].boundRect[1].X() = point.X();

			if (  point.Y() > viewVec[viewLoop].boundRect[1].Y()  )
				viewVec[viewLoop].boundRect[1].Y() = point.Y();

		}
		//P2DPRINT(viewVec[viewLoop].boundRect[0]);
		//P2DPRINT(viewVec[viewLoop].boundRect[1]);


		if  ( mxGetFieldNumber( matlabViewVec, cameraFieldStr.c_str() ) < 0 ) 
		{
			string errMssg("missing camera field in viewVec from Matlab.");
			throw errMssg;	
		}

		const mxArray* matlabCamera =				
			mxGetField( matlabViewVec, viewLoop, cameraFieldStr.c_str() );

		viewVec[viewLoop].camera.SetFromMatlabVar( matlabCamera, 0 );

	}

}
#endif


void SilhouetteSet::PushBack( const SilhouetteView & silView )
{
	viewVec.push_back( silView );
}


#ifdef MATLAB_MEX_FILE
void SilhouetteSet::SetFromMatlabViewVec( const mxArray* matlabViewVec )
{

	//mexPrintf("dfdfr\n");

	string cameraFieldStr = "camera";
	string boundaryFieldStr = "boundary";
	const int numViews = mxGetNumberOfElements( matlabViewVec );
	viewVec.resize(numViews);

	for(int viewLoop = 0; viewLoop < numViews; viewLoop++)
	{

		//INTEGERPRINT(viewLoop);

		if  ( mxGetFieldNumber( matlabViewVec, boundaryFieldStr.c_str() ) < 0 ) 
		{
			string errMssg("missing boundary field in viewVec from Matlab.");
			throw errMssg;	
		}

		const mxArray* matlabBoundary =				
			mxGetField( matlabViewVec, viewLoop, boundaryFieldStr.c_str() );

		const int numVertices = mxGetN(matlabBoundary);

		//INTEGERPRINT(numVertices);

		viewVec[viewLoop].boundary.resize(numVertices);
		for( int vLoop = 0; vLoop < numVertices; vLoop++ )
		{

			viewVec[viewLoop].boundary[vLoop] = Vector2d(
				mxGetPr(matlabBoundary)[vLoop*2+0],
				mxGetPr(matlabBoundary)[vLoop*2+1] );


		}


		viewVec[viewLoop].SetUp(); //form convex hull, build map etc.

		viewVec[viewLoop].boundRect[0] = viewVec[viewLoop].boundary[0];
		viewVec[viewLoop].boundRect[1] = viewVec[viewLoop].boundary[0];

		for( int hullLoop = 0; 
			hullLoop < viewVec[viewLoop].NumConvexVertices(); hullLoop++ )
		{

			Vector2d point;
			ConvBoundPoint( hullLoop, viewLoop, point );

			if (  point.X() < viewVec[viewLoop].boundRect[0].X()  )
				viewVec[viewLoop].boundRect[0].X() = point.X();

			if (  point.Y() < viewVec[viewLoop].boundRect[0].Y()  )
				viewVec[viewLoop].boundRect[0].Y() = point.Y();

			if (  point.X() > viewVec[viewLoop].boundRect[1].X()  )
				viewVec[viewLoop].boundRect[1].X() = point.X();

			if (  point.Y() > viewVec[viewLoop].boundRect[1].Y()  )
				viewVec[viewLoop].boundRect[1].Y() = point.Y();

		}

		if  ( mxGetFieldNumber( matlabViewVec, cameraFieldStr.c_str() ) < 0 ) 
		{
			string errMssg("missing camera field in viewVec from Matlab.");
			throw errMssg;	
		}

		const mxArray* matlabCamera =				
			mxGetField( matlabViewVec, viewLoop, cameraFieldStr.c_str() );

		viewVec[viewLoop].camera.SetFromMatlabVar( matlabCamera, 0 );

	}
}


#endif

//
//double  SilhouetteSet::ScaleOfInscribedConvProj( 
//		const std::vector<Wm3::Vector3d> pointVec, 
//		Wm3::Vector3d centrePoint ) const
//{
//
//	vector<double> scaleVec( NumViews() );
//
//	for(int viewLoop = 0; viewLoop < NumViews(); viewLoop++)
//	{
//		scaleVec[viewLoop] = viewVec[viewLoop].ScaleOfInscribedConvProj( 
//		pointVec, centrePoint);
//	}
//
//	sort( scaleVec.begin(),  scaleVec.end() );
//
//	return scaleVec[0];
//}
//
//
//




double SilhouetteSet::TotalArea() const
{
	double tot = 0;

	for(unsigned int viewLoop = 0; viewLoop < viewVec.size(); viewLoop++)
		tot += fabs( viewVec[viewLoop].SignedArea() );

	return tot;
}


#ifdef MATLAB_MEX_FILE
mxArray* SilhouetteSet::SetMatlabViewVec( ) const
{

	//mxArray* matlabViewVec = mxCreateDoubleMatrix(1,1,mxREAL);
	//mxGetPr(matlabViewVec)[0]=99;



	const char *field_names[] = {"camera", "boundary"};

	int dims[2];
	dims[0] = 1;
	dims[1] = viewVec.size();

	mxArray* matlabViewVec = mxCreateStructArray(2, dims, 2, field_names);

	for(unsigned int viewLoop = 0; viewLoop < viewVec.size(); viewLoop++)
	{
		mxArray *cameraField;
		cameraField = viewVec[viewLoop].camera.SetMatlabVar(  );
		mxSetFieldByNumber( matlabViewVec, viewLoop, mxGetFieldNumber(matlabViewVec,"camera"), cameraField );

		mxArray *boundaryField = 
			mxCreateDoubleMatrix(2,viewVec[viewLoop].boundary.size() ,mxREAL);
		for( unsigned int vv = 0; vv < viewVec[viewLoop].boundary.size();++vv)
		{
			mxGetPr(boundaryField)[vv*2+0] = viewVec[viewLoop].boundary[vv].X();
			mxGetPr(boundaryField)[vv*2+1] = viewVec[viewLoop].boundary[vv].Y();
			mxSetFieldByNumber( matlabViewVec, viewLoop, mxGetFieldNumber(matlabViewVec,"boundary"), boundaryField );
		}


	}

	return matlabViewVec;
}
#endif




//double SilhouetteSet::PoseOpt( SilhouetteSet & silsetOther, 
//							  Wm3::Matrix4d orientMat,
//							  std::vector<double> & vecOpt,			
//							  const int maxIterations,
//							  const double tol,
//							  const int propRedRange,
//							  const double propRedThresh )
//{
//
//	EtOrthoErrFn efOrtho( this,  &silsetOther );
//	EtPerspErrFn efPersp( this,  &silsetOther );
//
//
//	const Matrix4d RbtAlign = 
//		momentAlign.Inverse() * 
//		orientMat * silsetOther.momentAlign;
//
//	const Matrix3d R(
//		RbtAlign(0,0), RbtAlign(0,1), RbtAlign(0,2), 
//		RbtAlign(1,0), RbtAlign(1,1), RbtAlign(1,2), 
//		RbtAlign(2,0), RbtAlign(2,1), RbtAlign(2,2) );
//
//	Quaterniond quat;
//	quat.FromRotationMatrix(R);
//
//	vector<double> vecInit(7);
//
//	vecInit[0] = quat.X();
//	vecInit[1] = quat.Y();
//	vecInit[2] = quat.Z();
//	vecInit[3] = quat.W(); //this is the original ordering used in Matlab -- we need to be consitent with this
//	vecInit[4] = RbtAlign(0,3);
//	vecInit[5] = RbtAlign(1,3);
//	vecInit[6] = RbtAlign(2,3);
//
//	GslLevenbergMarquardt lm( &efOrtho );	
//	lm.SetTolF( tol );	
//	lm.SetMaxIter( maxIterations );
//	lm.SetPropRedThresh(propRedThresh);
//	lm.SetPropRedRange(propRedRange);
//	vecOpt.resize( efOrtho.NumParams()  );	
//	lm.Minimize( vecInit, vecOpt );
//	//throw(string("thrown outta here"));
//	const double etErrPersp = efPersp.RmsErr(vecOpt);
//	return  etErrPersp;
//}
//
//
//
//double SilhouetteSet::DirtRobustPoseOpt( SilhouetteSet & silsetOther, 
//										const int numResidualsToUse,
//										Wm3::Matrix4d orientMat,
//										std::vector<double> & vecOpt,			
//										const int maxIterations,
//										const double tol,
//										const int propRedRange,
//										const double propRedThresh )
//{
//
//	EtOrthoDirtRobustErrFn efDirtRobust( this,  &silsetOther, numResidualsToUse );	
//	
//	const Matrix4d RbtAlign = 
//		momentAlign.Inverse() * 
//		orientMat * silsetOther.momentAlign;
//	const Matrix3d R(
//		RbtAlign(0,0), RbtAlign(0,1), RbtAlign(0,2), 
//		RbtAlign(1,0), RbtAlign(1,1), RbtAlign(1,2), 
//		RbtAlign(2,0), RbtAlign(2,1), RbtAlign(2,2) );
//	Quaterniond quat;
//	quat.FromRotationMatrix(R);
//	vector<double> vecInit(7);
//	vecInit[0] = quat.X();
//	vecInit[1] = quat.Y();
//	vecInit[2] = quat.Z();
//	vecInit[3] = quat.W(); //this is the original ordering used in Matlab -- we need to be consitent with this
//	vecInit[4] = RbtAlign(0,3);
//	vecInit[5] = RbtAlign(1,3);
//	vecInit[6] = RbtAlign(2,3);
//	GslLevenbergMarquardt lm( &efDirtRobust );	
//	lm.SetTolF( tol );	
//	lm.SetMaxIter( maxIterations );
//	lm.SetPropRedThresh(propRedThresh);
//	lm.SetPropRedRange(propRedRange);
//	//lm.SetVerbosityLevel( 999 );
//	vecOpt.resize( efDirtRobust.NumParams()  );			
//	lm.Minimize( vecInit, vecOpt );
//
//
//
//
//
//	const double etErrPersp = efDirtRobust.RmsErr(vecOpt);
//	//DOUBLEPRINT(etErrPersp);
//
//
//	
//
//	return  etErrPersp;
//}
//
//
//
//
