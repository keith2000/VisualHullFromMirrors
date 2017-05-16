#ifndef SILFUNCTIONS_H
#define SILFUNCTIONS_H


#include <vector>
//#include "SilhouetteView.h"
#include "Wm3Vector2.h"
#include "VcalCamera.h"
//#include "SilhouetteView.h"


//class SilhouetteView;

namespace Sil
{


	double Kross( const Wm3::Vector2d & a, const Wm3::Vector2d & b );

	Wm3::Matrix4d PoseInvert( const Wm3::Matrix4d & pose );

	bool  IsBehind( const Wm3::Matrix4d & pose, 
		const Wm3::Matrix4d &  otherPose  );


	void RadialSamples( const std::vector<Wm3::Vector2d> & poly, 
						const int numSamples, 
						std::vector<double> & radialDistVec );
	//equi-angle radial samples from origin
	//store min value if multiple values exist



	void PrintPoly(  const std::vector<Wm3::Vector2d> & poly );

	
//
//double EtErr( const std::vector< std::vector<SilhouetteView> > & silVecVec, 
//			 const int camA, const int blobA,
//			 const int camB, const int blobB );
//
//double EtErr( const std::vector< std::vector<SilhouetteView> > & silVecVec, 
//			 const int camA, const int blobA,
//			 const int camB, const int blobB, 
//			 const int camC, const int blobC );

}

#endif
