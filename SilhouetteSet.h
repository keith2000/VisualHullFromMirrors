#ifndef SILHOUETTESET_H
#define SILHOUETTESET_H

#include <vector>
#include "Wm3Vector2.h"
#include "Wm3Matrix4.h"
#include "Wm3LinearSystem.h"
//#include "QuaternionGridSet.h"
#include "SilhouetteView.h"
//#include "EtOrthoDirtRobustErrFn.h"

class EtScaleOrthoErrFn;
class EtOrthoErrFn;
class ConvexViewingEdges;
class ConvRimEst;
class PointInVisualHull;
class BallViewVecErr;

class SilhouetteSet
{
public:

	enum SpecialSilhouetteSetConstruction
	{
		silhouetteSetWithSingleSilhouette = 0,
		silhouetteSetWithUnorderedConvexVertices,
		constructWithoutLutOrOrtho,
		constructWithoutLut // value
	}; // this enum hopefully makes the user (client progammer) clear about what is being done by the constructor


	friend class EtOrthoErrFn;
	friend class ConvexViewingEdges;
	friend class ConvRimEst;
	friend class EtScaleOrthoErrFn;
	friend class PointInVisualHull;
	friend class BallViewVecErr;

	SilhouetteSet();	

#ifdef MATLAB_MEX_FILE
	SilhouetteSet( const mxArray* matlabViewVec );
	SilhouetteSet( const mxArray* matlabViewVec, int single );

	SilhouetteSet( const mxArray* matlabViewVec,
		SpecialSilhouetteSetConstruction flagVal );

	void SetFromMatlabViewVec( const mxArray* matlabViewVec );
	mxArray* SetMatlabViewVec(   ) const;

	void SetFromMatlabConvexUnorderedViewVec( const mxArray* matlabViewVec );

#endif


	double AlignFromMoments( SilhouetteSet & silsetOther, 
		Wm3::Matrix4d orientMat,
		const int maxIterations = 15,
		const double tol = 1e-6,
		const int propRedRange = 3,
		const double propRedThresh = 0.05);

	void ConstDepthRims(  std::vector<Wm3::Vector3d> & pointVec, 
		std::vector<int> & labelVec ) const;

	bool IsMomentMatch(  SilhouetteSet & silsetOther, 
		const double etThresh, 
		int & exitType,
		const int maxIterations = 15,
		const double tol = 1e-6,
		const int propRedRange = 3,
		const double propRedThresh = 0.05,
		const int numRandStartingPoints = 200 );


	bool IsQuatMatch( SilhouetteSet & silsetOther, 
		const int samplesPerAxis,
		const double etThresh, 
		int & exitType,
		const int maxIterations = 15,
		const double tol = 1e-6,
		const int propRedRange = 3,
		const double propRedThresh = 0.05,
		const int numRandStartingPoints = 200 );


	bool IsMatch( SilhouetteSet & silsetOther, 
		std::vector<Wm3::Matrix4d> orientVec,
		const double etThresh, 
		int & exitType,
		const int maxIterations = 15,
		const double tol = 1e-6,
		const int propRedRange = 3,
		const double propRedThresh = 0.05,
		const int numRandStartingPoints = 200 );

	double PoseOpt( SilhouetteSet & silsetOther, 
		Wm3::Matrix4d orientMat,
		std::vector<double> & vecOpt,			
		const int maxIterations = 15,
		const double tol = 1e-6,
		const int propRedRange = 3,
		const double propRedThresh = 5/100
		);


	double DirtRobustPoseOpt( SilhouetteSet & silsetOther, 		
		const int numResidualsToUse,
		Wm3::Matrix4d orientMat,
		std::vector<double> & vecOpt,			
		const int maxIterations = 15,
		const double tol = 1e-6,
		const int propRedRange = 3,
		const double propRedThresh = 5/100
		);

	bool IsMatch( SilhouetteSet & silsetOther, 
		Wm3::Matrix3d orientationMatrix,
		const double etThresh, 							
		const int maxIterations = 15,
		const double tol = 1e-6,
		const int propRedRange = 3,
		const double propRedThresh = 0.05,
		const int numRandStartingPoints = 200 );

	double MinimiseEtErrFromPose( SilhouetteSet & silsetOther, 
		Wm3::Matrix3d orientationMatrix,			
		const int maxIterations = 15,
		const double tol = 1e-6,
		const int propRedRange = 3,
		const double propRedThresh = 0.05 );

	double MinimiseEtErrFromPoseScale( SilhouetteSet & silsetOther, 
		Wm3::Matrix3d orientationMatrix,			
		const int maxIterations = 15,
		const double tol = 1e-6,
		const int propRedRange = 3,
		const double propRedThresh = 0.05 );


	void ConvBoundPoint( const int kInd, const int viewInd, Wm3::Vector2d & point ) const;
	int ConvBoundNumPoints( const int viewInd ) const;


	double MeanConvexOrthoArea(  ) const;

	int NumViews() const { return viewVec.size(); }


	double AngleAtEpipole( const int viewA, const int viewB ) const;
	bool IsEpipoleInBoundRect( const int viewA, const int viewB ) const;


	void EtResiduals( const int indViewA, const int indViewB, double * resVec ) const;


	SilhouetteView& View( const int ind ) { return viewVec[ind]; };
	SilhouetteView View( const int ind ) const { return viewVec[ind]; };


	void FormConvexHull();


	void ApproxCentrePoint( Wm3::Vector3d & centrePoint ) const;
	void LmCentrePoint( const double tol, 
		const long maxIterations, Wm3::Vector3d & centrePoint ) const;

	void CreateOrthoBoundaries( const double lmTol = 1e-4, const long lmMfe = 50 );

	void CreateLuts( const int numAngles = 100 );

	void SetMomentAlign( double * data );
	void SetMomentAlignFromQuatTran( double * data );


	void SetCameraPose( const int camInd, const Wm3::Matrix4d & pose );
	void GetCameraPose( const int camInd,  Wm3::Matrix4d & pose ) const;
	void GetCamera( const int camInd, VcalCamera &cam ) const;    
	void SetCamera( const int camInd, const VcalCamera &cam );    

	void SetCameraEfl( const int camInd, const double eflVal );
	void SetCameraU0( const int camInd, const double u0Val );
	void SetCameraV0( const int camInd, const double v0Val );
	double GetCameraEfl( const int camInd ) const;
	double GetCameraU0( const int camInd ) const;
	double GetCameraV0( const int camInd ) const;


	double ScaleOfInscribedConvProj( 
		const std::vector<Wm3::Vector3d> & pointVec, 
		const Wm3::Vector3d & centrePoint ) const;
	//how much can the convex hull of the projected points be scaled about the centre point


	bool IsInsideVisualHull( const Wm3::Vector3d & point ) const;



	void SetVvCentrePoint(); //set member data vvCentrePoint using approx triangulatiton


	double  WeakScaleOfInscribedConvProj( 
		const std::vector<Wm3::Vector3d> & pointVec, 
		const Wm3::Vector3d & centrePoint,
		const double theta, const double phi, 
		std::vector<double> & bestDerivVec ) const;



	Wm3::Vector3d vvCentrePoint;

	void PushBack( const SilhouetteView & silView );

	double TotalArea() const;

	Wm3::Matrix4d MomentAlign(){return momentAlign;}

private:

	std::vector<SilhouetteView> viewVec;

	//public:
	Wm3::Matrix4d momentAlign; //matrix to align principal axes of stone with x,y,z axes
	//can be estimated from vemh



};

#endif 
