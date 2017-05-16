#ifndef SILHOUETTEVIEW_H
#define SILHOUETTEVIEW_H

#include <vector>
#include <list>
#include <map>
#include "Wm3Vector2.h"
#include "VcalCamera.h"
#include "Wm3ContConvexHull2.h"

class EtScaleOrthoErrFn;
class SilhouetteSet;
class EtOrthoErrFn;
class ConvexViewingEdges;
class ConvRimEst;
class PointInVisualHull;
class BallViewVecErr;
//class VcalCamera;

class SilhouetteView
{
public:

	friend class SilhouetteSet;
	friend class EtOrthoErrFn;
	friend class ConvexViewingEdges;
	friend class ConvRimEst;
	friend class PointInVisualHull;
	friend class EtScaleOrthoErrFn;
	friend class BallViewVecErr;

#ifdef MATLAB_MEX_FILE
	SilhouetteView( const mxArray* matlabPoly );
#endif

	SilhouetteView();


	void SetUp();
	double SignedConvexArea() const;
	double SignedArea() const;

	void ConvexSilhouetteCentroid(Wm3::Vector2d & centre) const;

	void RemoveCollinearConvexHullVertices( const double tol );
	void PrintConvBoundary() const;

	void CreateConstDepthConvRim( const Wm3::Vector3d centrePoint, 
		std::vector<Wm3::Vector3d> & rimVec ) const;


	void AddVertex( const Wm3::Vector2d & vertex ) { boundary.push_back(vertex); }


	void EpipolarTangencies( const Wm3::Vector2d & epipole,
		Wm3::Vector2d & tA, Wm3::Vector2d & tB ) const;


	void LmTriangulate(
		const Wm3::Vector2d & imgPointA, 
		const Wm3::Vector2d & imgPointB, 
		const VcalCamera & cameraA, 
		const VcalCamera & cameraB, 
		Wm3::Vector3d & approxPoint ) const;

	bool IsEpipoleInBoundRect( const Wm3::Vector2d & epipole ) const;

	bool IsEpipoleInBoundRect( const SilhouetteView & viewOther, 
		const Wm3::Matrix4d & RbtAdjust ) const;


	double AngleAtEpipole( const Wm3::Vector2d & epipole ) const;


	double AngleAtEpipole(const SilhouetteView & viewOther, 
		const Wm3::Matrix4d & RbtAdjust ) const;



	void EtResiduals(const SilhouetteView & viewOpp,
		const Wm3::Matrix4d & RbtAdjust,
		double * resVec ) const;

	void EtSimilarityResiduals(const SilhouetteView & viewOpp,
		const Wm3::Matrix4d & RbtScaleAdjust,
		bool & isAnEpipoleInside,
		double * resVec ) const;

	void EtResiduals(const SilhouetteView & viewOpp,
		const Wm3::Matrix4d & RbtAdjust,
		bool & isAnEpipoleInside,
		double * resVec ) const;
	//newer version uses map and reports if epi is inside either any case


	void CreateOrthoBoundary( const Wm3::Vector3d centrePoint );

	void GeneratePointIndFromAngLut( const int numAngles = 100 );


	int LutSize() const { return pointIndFromAngLut.size(); }

	int LutListLen( const int theEntry ) const 
	{ return pointIndFromAngLut[theEntry].size(); }


	Wm3::Vector2d OrthoConvHullVertex( const int ind ) const;

	Wm3::Vector2d ConvHullVertex( const int ind ) const;


	Wm3::Vector2d ConvHullNiVertex( const int ind ) const;

	int NumConvexVertices() const { return convHullIndVec.size(); }

	void FormConvexHull();

	void CopyConvexHullFromBoundary();

	void CreateNiBoundary();


	void FrontierErr(const SilhouetteView & viewOpp,
		const Wm3::Matrix4d & RbtAdjust,
		const Wm3::Vector3d & frontierPointA,
		const Wm3::Vector3d & frontierPointB,
		double * resVec ) const;


	void EstFrontierPoints(const SilhouetteView & viewOpp,
		const Wm3::Matrix4d & RbtAdjust,
		Wm3::Vector3d & frontierPointA,
		Wm3::Vector3d & frontierPointB) const;


	void ApproxTriangulate( 		   const Wm3::Vector2d & imgPointA, 
		const Wm3::Vector2d & imgPointB, 
		const VcalCamera & cameraA, 
		const VcalCamera & cameraB, 
		Wm3::Vector3d & approxPoint ) const;


	//void JitterBoundary( const double halfwidth );

	void CreateModGradMap();
	void CreateModGradLut();

	Wm3::Vector2d VertexFromDirection( 
		const double xVal,  const double yVal  ) const;

	int IndexFromDirection( 
		const double xVal,  const double yVal  ) const;

	bool IsTangency( const int vertexIndex,
		const Wm3::Vector2d & epipole ) const;

	void TangenciesFromMap( const Wm3::Vector2d & epipole,
		Wm3::Vector2d & tLeft,
		Wm3::Vector2d & tRight ) const;




	void JitterBoundary( const double noiseLevel );

	void PrintOrthoBoundary() const;

	double SignedConvexOrthoArea() const;

	double ScaleOfInscribedConvProj( 
		const std::vector<Wm3::Vector3d> & pointVec, 
		const Wm3::Vector3d & centrePoint ) const;
	//how much can the convex hull of the projected points be scaled about the centre point

	bool Contains( const Wm3::Vector2d & point ) const;

	double RadialSampScaleOfInscribedConvProj( const int numRadialSamples,
		const std::vector<Wm3::Vector3d> & pointVec, 
		const Wm3::Vector3d & centrePoint ) const;

	//double WeakPerspScaleInscribedConvProj( 
	//	const std::vector<Wm3::Vector3d> & origPointVec, 
	//	const Wm3::Vector3d & brilCentrePoint,
	//	const double theta, const double phi, 
	//	const Wm3::Vector3d & vvCentrePoint) const;

	double WeakPerspScaleInscribedConvProj( 
		const std::vector<Wm3::Vector3d> & origPointVec, 
		const Wm3::Vector3d & brilCentrePoint,
		const double theta, const double phi, 
		const Wm3::Vector3d & vvCentrePoint, 
		std::vector<double> & derivVec) const;


	void SetBlobView( const mxArray* matlabVar );
	void SetBlobView( const std::vector<Wm3::Vector2d> & boundaryVal,
		const VcalCamera & cameraVal, const Wm3::Vector2d & offset  );


	void SetBoundingRectangle();


	bool DoesTouchBorder( const bool doMatlabImageOrigin) const;

private:

	bool IsFurtherClockwise(const Wm3::Vector2d& A, const Wm3::Vector2d& B) const;

	double ModifiedGrad( const double xVal, const double yVal ) const;
	//monotonic function of atan2 that is faster to compute



	VcalCamera camera;
	std::vector<Wm3::Vector2d> boundary;
	std::vector<int> convHullIndVec;

	std::vector<Wm3::Vector2d> orthoBoundary;
	std::vector<Wm3::Vector2d> niBoundary;

	Wm3::Vector2d boundRect[2];


	Wm3::Vector2d centrePoint;
	std::vector< std::list<int> > pointIndFromAngLut;

	std::map< double, int > pointIndFromModGradMap;

	std::vector<double> pointIndFromModGradLut;



};

#endif 
