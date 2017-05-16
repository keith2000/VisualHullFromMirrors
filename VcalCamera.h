#ifndef VCALCAMERA_H
#define VCALCAMERA_H


#ifdef MATLAB_MEX_FILE
#include "mex.h"
#endif


#include <string>
#include "Wm3Vector2.h"
#include "Wm3Matrix4.h"
#include "Wm3Matrix3.h"
#include "Wm3Vector4.h"
#include "Wm3Vector3.h"
#include "SilFunctions.h"

class SilhouetteSet;
class SilhouetteView;
class EtOrthoErrFn;
class ConvexViewingEdges;
class ConvRimEst;
class EtScaleOrthoErrFn;
class PointInVisualHull;

class VcalCamera
{
public:
	
	friend class EtScaleOrthoErrFn;
	friend class SilhouetteSet;
	friend class SilhouetteView;
	friend class EtOrthoErrFn;
	friend class ConvexViewingEdges;
	friend class ConvRimEst;
	friend class PointInVisualHull;

	VcalCamera();	
	virtual ~VcalCamera();
	
#ifdef MATLAB_MEX_FILE
	VcalCamera( const mxArray* matCam, const int matIndex);
	//doPpMinusOne: set to true to subtract one from principal point coordinates to convert from matlab to C-style indexing
	void SetFromMatlabVar( const mxArray* matCam, const int matIndex, const bool doPpMinusOne = false  );
	//void VcalCamera::SetFromMatlabVar( const mxArray* matCam, const int matIndex, const bool doPpMinusOne )

	mxArray* SetMatlabVar( ) const ;
#endif
	
	//void SetEfl( const double eflVal ) { efl = eflVal; }
	//void SetU0( const double u0Val ) { u0 = u0Val; }
	//void SetV0( const double v0Val ) { v0 = v0Val; }
	//void SetKappa1( const double kappa1Val ) { kappa1 = kappa1Val; }
	
	double& Efl() { return efl; }
	double Efl() const { return efl; }
	
	double& U0() { return u0; }
	double U0() const { return u0; }
	
	long& Width() { return width; }
	long Width() const { return width; }

	long& Height() { return height; }
	long Height() const { return height; }
	
	double& V0() { return v0; }
	double V0() const { return v0; }
	
	double& Kappa1() { return kappa1; }
	double Kappa1() const { return kappa1; }
	
	Wm3::Matrix4d & Pose() { return pose; }
	Wm3::Matrix4d Pose() const { return pose; }
	
	void ImageToNormalisedImage  ( const Wm3::Vector2d & p_i, Wm3::Vector2d & p_ni) const;
	
	Wm3::Vector2d  ImageToNormalisedImage ( const Wm3::Vector2d & p_i ) const;
	
	
	void NormalisedImageToImage  ( const Wm3::Vector2d & p_ni, Wm3::Vector2d & p_i) const;
	
	void UndistortedToDistorted  ( const Wm3::Vector2d & pUnistorted, Wm3::Vector2d & pDistorted) const;
	void DistortedToUndistorted  ( const Wm3::Vector2d & pDistorted, Wm3::Vector2d & pUnistorted) const;

	//bool AreFacing( const VcalCamera & otherCam ) const;
	bool AreFacing( const Wm3::Matrix4d &  otherPose ) const;
	//true if cameras are facing one another
	//false if one or both cameras is behind the other
	//note that it is possible for both cameras to be behind each other

	bool IsBehind( const Wm3::Matrix4d &  otherPose ) const;
	
	//Wm3::Matrix4d PoseInverse() const;
	void CreateK( Wm3::Matrix3d & K ) const;
	void CreateInvK( Wm3::Matrix3d & invK ) const;
	
	void CreateR( Wm3::Matrix3d & R ) const;
	void Create_t( Wm3::Vector3d & t ) const;
	void CreateKR( Wm3::Matrix3d & KR ) const;
	void CreateKt( Wm3::Vector3d & Kt ) const;
	
	void CreateKRt( Wm3::Matrix4d & KRt ) const;
	
	//double CameraInternals::CubeRoot(const double x);
	
	
	void Epipole( const VcalCamera & otherCam, Wm3::Matrix4d RbtAdjust,
		Wm3::Vector2d & epipole  ) const;
	
	void WorldToImage( const Wm3::Vector3d & pWorld,  
		Wm3::Vector2d & pImage ) const;

	void WeakWorldToImage( const Wm3::Vector3d & pWorld,  
		const Wm3::Vector3d & centrePoint,   
		Wm3::Vector2d & pImage ) const;

	void Print();

	void CameraToWorld( const Wm3::Vector3d & pCam,  Wm3::Vector3d & pWorld ) const;
	void WorldToCamera( const Wm3::Vector3d & pWorld,  Wm3::Vector3d & pCam ) const;
		
	
	
private:
	double efl;
	double u0;
	double v0;
	double kappa1;
	Wm3::Matrix4d pose;
	std::string serial;
	long width;
	long height;
	
	double CubeRootFn(const double x) const;
	
};

#endif 
