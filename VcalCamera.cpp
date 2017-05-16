#include<iostream>
#include<string>
#include<sstream>
#include<cmath>
#include "VcalCamera.h"

using namespace std;
using namespace Wm3;

#define COMPLEXPRINT(str) mexPrintf("%% %s: %+g %+gi\n", #str, real(str), imag(str) );
#define DOUBLEPRINT(str) mexPrintf("%% %s: %g\n", #str, str );
#define BOOLPRINT(str) mexPrintf("%% %s: %s\n", #str, str?"true":"false" );
#define INTEGERPRINT(str) mexPrintf("%% %s: %d\n", #str, str );
#define P2DPRINT(str) mexPrintf(" %s= [%g; %g]\n", #str, str.X(), str.Y() );
#define P3DPRINT(str) mexPrintf("%% %s: [%g; %g; %g]\n", #str, str.X(), str.Y() , str.Z() );
#define M4PRINT(str) mexPrintf("%% %s= [[%g,%g,%g,%g];[%g,%g,%g,%g];[%g,%g,%g,%g];[%g,%g,%g,%g]]\n", #str, str(0,0), str(0,1), str(0,2), str(0,3), str(1,0), str(1,1), str(1,2), str(1,3), str(2,0), str(2,1), str(2,2), str(2,3), str(3,0), str(3,1), str(3,2), str(3,3) );
#define M3PRINT(str) mexPrintf("%% %s= [[%g,%g,%g];[%g,%g,%g];[%g,%g,%g]]\n", #str, str(0,0), str(0,1), str(0,2), str(1,0), str(1,1), str(1,2), str(2,0), str(2,1), str(2,2) );
#define MSASSERT(str) if (!(str)) throw(string("failed assertion:") + string(#str));



bool VcalCamera::AreFacing( const Wm3::Matrix4d &  otherPose ) const
{

	//3rd row of pose matrix gives direction

	//const double dotProduct = 
	//	pose(2, 0) * otherPose(2, 0) +
	//	pose(2, 2) * otherPose(2, 1) +
	//	pose(2, 1) * otherPose(2, 2);

	bool areFacing = true;

	const Vector4d camCentre( 0, 0, 0, 1);


	const Vector4d otherInThis = pose * Sil::PoseInvert(otherPose) * camCentre;
	if ( otherInThis.Z() < 0 )
		areFacing = false;

	const Vector4d thisInOther = otherPose * Sil::PoseInvert(pose) * camCentre;
	if ( thisInOther.Z() < 0 )
		areFacing = false;




	
	
	return areFacing;
}


bool VcalCamera::IsBehind( const Wm3::Matrix4d &  otherPose ) const
{	
	const Vector4d camCentre( 0, 0, 0, 1);

	const Vector4d thisInOther = 
		otherPose * Sil::PoseInvert(pose) * camCentre;
	
	return   thisInOther.Z() < 0;
			
}



/*///////////////
bool VcalCamera::AreFacing( const VcalCamera & otherCam ) const
{

	//3rd row of pose matrix gives direction
//THIS IS JUNK!
	const double dotProduct = 
		pose(2, 0) * otherCam.pose(2, 0) +
		pose(2, 2) * otherCam.pose(2, 1) +
		pose(2, 1) * otherCam.pose(2, 2);
	
	
	return dotProduct > 0;
}
/*///////////////


void VcalCamera::Epipole( const VcalCamera & otherCam, Wm3::Matrix4d RbtAdjust,
						 Wm3::Vector2d & epipole  ) const
{
	
	const Vector4d otherCamCenWorld = 
		RbtAdjust * otherCam.pose.Inverse() * 
		Vector4d( 0,0,0,1 );
	
	
	WorldToImage( Vector3d( 
		otherCamCenWorld.X(), 
		otherCamCenWorld.Y(), 
		otherCamCenWorld.Z() ), epipole );
	
	
}

void VcalCamera::CameraToWorld( const Wm3::Vector3d & pCam,  Wm3::Vector3d & pWorld ) const
{
	const Vector4d pCamHomog( pCam.X(), pCam.Y(), pCam.Z(), 1 );
	
	const Vector4d pWorldHomog = pose.Inverse() * pCamHomog;

	pWorld.X() = pWorldHomog.X();
	pWorld.Y() = pWorldHomog.Y();
	pWorld.Z() = pWorldHomog.Z();
}



void VcalCamera::WorldToCamera( const Wm3::Vector3d & pWorld,  Wm3::Vector3d & pCam ) const
{
	const Vector4d pWorldHomog( pWorld.X(), pWorld.Y(), pWorld.Z(), 1 );
	
	const Vector4d pCamHomog = pose * pWorldHomog;

	pCam.X() = pCamHomog.X();
	pCam.Y() = pCamHomog.Y();
	pCam.Z() = pCamHomog.Z();

}

void VcalCamera::WeakWorldToImage( const Wm3::Vector3d & pWorld,  
		const Wm3::Vector3d & centrePoint,   
		Wm3::Vector2d & pImage ) const
{
	Vector3d pCam, pCenCam;

	WorldToCamera( pWorld,  pCam );
	WorldToCamera( centrePoint,  pCenCam );

	const Vector2d pNi( 
		pCam.X()/pCenCam.Z(), 
		pCam.Y()/pCenCam.Z() );

	NormalisedImageToImage( pNi, pImage);
}

		
void  VcalCamera::Print()
{

	mexPrintf("%7.2f %7.2f %7.2f %10.2f| %15.1f\n", pose(0,0), pose(0,1), pose(0,2), pose(0,3), efl );
	mexPrintf("%7.2f %7.2f %7.2f %10.2f| %15.1f\n", pose(1,0), pose(1,1), pose(1,2), pose(1,3), u0 );
	mexPrintf("%7.2f %7.2f %7.2f %10.2f| %15.1f\n", pose(2,0), pose(2,1), pose(2,2), pose(2,3), v0 );
}

void VcalCamera::WorldToImage( const Wm3::Vector3d & pWorld,  
							  Wm3::Vector2d & pImage ) const
{
	
	
	const Vector4d pWorldHomog( pWorld.X(), pWorld.Y(), pWorld.Z(), 1 );
	
	const Vector4d pCamHomog = pose * pWorldHomog;
	
	const Vector2d pNi( 
		pCamHomog.X()/pCamHomog.Z(), 
		pCamHomog.Y()/pCamHomog.Z() );
	
	NormalisedImageToImage( pNi, pImage);
	
}





VcalCamera::VcalCamera()
{
	
}

VcalCamera::~VcalCamera()
{
	
}


#ifdef MATLAB_MEX_FILE
VcalCamera::VcalCamera( const mxArray* matCam, const int matIndex )
{
	SetFromMatlabVar( matCam, matIndex );
}
#endif

#ifdef MATLAB_MEX_FILE
mxArray* VcalCamera::SetMatlabVar(   ) const 
{

	const char *field_names[] = {"efl", "u0", "v0", "kappa1", 
		"pose", "width", "height", "serial"};
	
	int dims[2] = {1, 1 };
	mxArray* matCam = mxCreateStructArray(2, dims, 8, field_names);

	mxArray *eflField = mxCreateDoubleMatrix(1,1,mxREAL);
	mxGetPr(eflField)[0] = efl;
	mxSetFieldByNumber( matCam, 0, mxGetFieldNumber(matCam,"efl"), eflField );
    
	mxArray *u0Field = mxCreateDoubleMatrix(1,1,mxREAL);
	mxGetPr(u0Field)[0] = u0;
	mxSetFieldByNumber( matCam, 0, mxGetFieldNumber(matCam,"u0"), u0Field );
    
	mxArray *v0Field = mxCreateDoubleMatrix(1,1,mxREAL);
	mxGetPr(v0Field)[0] = v0;
	mxSetFieldByNumber( matCam, 0, mxGetFieldNumber(matCam,"v0"), v0Field );

	mxArray *kappa1Field = mxCreateDoubleMatrix(1,1,mxREAL);
	mxGetPr(kappa1Field)[0] = kappa1;
	mxSetFieldByNumber( matCam, 0, mxGetFieldNumber(matCam,"kappa1"), kappa1Field );
    	    	
	mxArray *widthField = mxCreateDoubleMatrix(1,1,mxREAL);
	mxGetPr(widthField)[0] = double(width);
	mxSetFieldByNumber( matCam, 0, mxGetFieldNumber(matCam,"width"), widthField );
    
	mxArray *heightField = mxCreateDoubleMatrix(1,1,mxREAL);
	mxGetPr(heightField)[0] = double(height);
	mxSetFieldByNumber( matCam, 0, mxGetFieldNumber(matCam,"height"), heightField );
    	
	mxArray *serialField = mxCreateString(serial.c_str());
	mxSetFieldByNumber(matCam,0,mxGetFieldNumber(matCam,"serial"),serialField);

	mxArray *poseField = mxCreateDoubleMatrix(4,4,mxREAL);
	for( int rr = 0; rr < 4;++rr)
		for( int cc = 0; cc < 4;++cc)
	mxGetPr(poseField)[rr+cc*4] = pose(rr,cc);
	mxSetFieldByNumber( matCam, 0, mxGetFieldNumber(matCam,"pose"), poseField );
    

	return matCam;

}
#endif


 
#ifdef MATLAB_MEX_FILE
void VcalCamera::SetFromMatlabVar( const mxArray* matCam, const int matIndex, const bool doPpMinusOne )
{
	
	//INTEGERPRINT(matIndex);
	MSASSERT( matIndex <  mxGetNumberOfElements(matCam) );
	MSASSERT( matIndex >=  0 );
	
	const string eflFieldStr = "efl";
	const string u0FieldStr = "u0";
	const string v0FieldStr = "v0";
	const string kappa1FieldStr = "kappa1";	
	const string poseFieldStr = "pose";	

	const string widthFieldStr = "width";	
	const string heightFieldStr = "height";	
	const string serialFieldStr = "serial";	
	
	MSASSERT( mxGetFieldNumber( matCam, eflFieldStr.c_str() ) >= 0 );
	MSASSERT( mxGetFieldNumber( matCam, u0FieldStr.c_str() ) >= 0 );
	MSASSERT( mxGetFieldNumber( matCam, v0FieldStr.c_str() ) >= 0 );
	MSASSERT( mxGetFieldNumber( matCam, kappa1FieldStr.c_str() ) >= 0 );
	MSASSERT( mxGetFieldNumber( matCam, widthFieldStr.c_str() ) >= 0 );
	MSASSERT( mxGetFieldNumber( matCam, heightFieldStr.c_str() ) >= 0 );
	MSASSERT( mxGetFieldNumber( matCam, serialFieldStr.c_str() ) >= 0 );
	MSASSERT( mxGetFieldNumber( matCam, poseFieldStr.c_str() ) >= 0 );
		


	width = long( mxGetPr((mxGetField( matCam, matIndex, widthFieldStr.c_str() ) ) )[0] );
	height = long( mxGetPr((mxGetField( matCam, matIndex, heightFieldStr.c_str() ) ) )[0] );



	mxArray * serialMat = mxGetField( matCam, matIndex, serialFieldStr.c_str() );
	const int buflen = mxGetNumberOfElements(serialMat) * sizeof(mxChar) + 1;
	char * str = new char[buflen];	
	mxGetString( serialMat, str, buflen);		
	serial = string(str);
	delete [] str;		
	//serial = "edfewfwe";




	efl = mxGetPr((mxGetField( matCam, matIndex, eflFieldStr.c_str() ) ) )[0];

	//DOUBLEPRINT(efl);

	u0 = mxGetPr((mxGetField( matCam, matIndex, u0FieldStr.c_str() ) ) )[0];
	//DOUBLEPRINT(u0);
	v0 = mxGetPr((mxGetField( matCam, matIndex, v0FieldStr.c_str() ) ) )[0];
	//DOUBLEPRINT(v0);
	kappa1 = mxGetPr((mxGetField( matCam, matIndex, kappa1FieldStr.c_str() ) ) )[0];

	//DOUBLEPRINT(kappa1);
	
	const mxArray* matPose = mxGetField( matCam, matIndex, poseFieldStr.c_str() );
	
	const double * data = mxGetPr(matPose);
	pose = Matrix4d( 
		data[0], data[4],  data[8], data[12],
		data[1], data[5],  data[9], data[13],
		data[2], data[6], data[10], data[14],
		data[3], data[7], data[11], data[15] );
	//M4PRINT(pose);
	
}
#endif


Wm3::Vector2d  VcalCamera::ImageToNormalisedImage ( const Wm3::Vector2d & p_i ) const
{
	
	Vector2d p_ni;
	ImageToNormalisedImage( p_i, p_ni);
	return p_ni;
}


void VcalCamera::ImageToNormalisedImage 
( const Wm3::Vector2d & p_i, Wm3::Vector2d & p_ni) const
{		
	Vector2d pDistorted( p_i.X() - u0,  p_i.Y() - v0 ); //move centroid to principal point		
	Vector2d pUnistorted;	
	if ( kappa1 == 0)
	{
		pUnistorted = pDistorted;
	}
	else
	{
		DistortedToUndistorted( pDistorted, pUnistorted );   
	}		
	
	p_ni = pUnistorted / efl; //divide by the effective focal length to get the image on the z=1 plane		
}




void VcalCamera::NormalisedImageToImage 
( const Wm3::Vector2d & p_ni, Wm3::Vector2d & p_i) const
{	
	const Vector2d pUnistorted = p_ni * efl;	
	Vector2d pDistorted;	
	if ( kappa1 == 0)
	{
		pDistorted = pUnistorted;
	}
	else
	{
		UndistortedToDistorted( pUnistorted, pDistorted );
		Vector2d pUnistortedRecomputed;
		DistortedToUndistorted( pDistorted, pUnistortedRecomputed );
		
		
		if ( fabs((pUnistortedRecomputed - pUnistorted).Length()) > 1e-10 )
		{
			
			
			stringstream strStream;
			
			strStream << "pUnistortedRecomputed != pUnistorted, diff: ";
			strStream <<  fabs((pUnistortedRecomputed - pUnistorted).Length());
			
			// the overhead to check this is small compared to the UndistortedToDistorted computation
			//string errMssg("pUnistortedRecomputed != pUnistorted");
			
			string errMssg = strStream.str();
			
			//cout << errMssg << endl;
			//throw errMssg;
		}
	}	
	p_i = Vector2d( pDistorted.X() + u0, pDistorted.Y() + v0 ); 	
	
}



void VcalCamera::CreateR( Wm3::Matrix3d & R ) const
{	
	for( int rr=0;rr<3;rr++)
		for( int cc=0;cc<3;cc++)
			R(rr,cc) = pose(rr,cc);
}

void VcalCamera::Create_t( Wm3::Vector3d & t ) const
{	
	t.X() = pose(0,3);
	t.Y() = pose(1,3);
	t.Z() = pose(2,3);
}



void VcalCamera::CreateKR( Wm3::Matrix3d & KR ) const
{
	Matrix3d K; 
	CreateK(K);

	Matrix3d R; 
	CreateR(R);
	
	KR = K*R;
}

void VcalCamera::CreateKt( Wm3::Vector3d & Kt ) const
{	
	Matrix3d K; 
	CreateK(K);

	Vector3d t;
	Create_t(t);

	Kt = K*t;
}

void VcalCamera::CreateK( Wm3::Matrix3d & K ) const
{
	
	if ( kappa1 != 0 )
	{			
		throw string("Cannot form matrix K with nonzero kappa1");
	}


	K(0,0) = efl;
	K(0,1) = 0;
	K(0,2) = u0;

	K(1,0) = 0;
	K(1,1) = efl;
	K(1,2) = v0;


	K(2,0) = 0;
	K(2,1) = 0;
	K(2,2) = 1;


}

void VcalCamera::CreateInvK( Wm3::Matrix3d & invK ) const
{	
	if ( kappa1 != 0 )
	{			
		throw string("Cannot form matrix K with nonzero kappa1");
	}

	invK(0,0) = 1/efl;
	invK(0,1) = 0;
	invK(0,2) = -u0/efl;

	invK(1,0) = 0;
	invK(1,1) = 1/efl;
	invK(1,2) = -v0/efl;


	invK(2,0) = 0;
	invK(2,1) = 0;
	invK(2,2) = 1;


}

void VcalCamera::DistortedToUndistorted 
( const Wm3::Vector2d & pDistorted, 
 Wm3::Vector2d & pUnistorted) const
{
	const double rSquared = pDistorted.X() * pDistorted.X() + 
		pDistorted.Y() * pDistorted.Y();	
	
	
	pUnistorted = Vector2d( pDistorted.X() + pDistorted.X() * ( kappa1 * rSquared ),
		pDistorted.Y() + pDistorted.Y() * ( kappa1 * rSquared ) );	
	
}




void VcalCamera::UndistortedToDistorted ( const Wm3::Vector2d & pUnistorted,
										 Wm3::Vector2d & pDistorted ) const
										 
{
	const double SQRT3 = 1.732050807568877293527446341505872366943;    
    const double Ru = sqrt( pUnistorted.X() * pUnistorted.X() + pUnistorted.Y() * pUnistorted.Y() );	
    const double c = 1 / kappa1;
    const double d = -c * Ru;	
    const double Q = c / 3;
    const double R = -d / 2;    
	double D = Q*Q*Q + R*R;	
	double Rd;	
    if (D >= 0) 
	{		/* one real root */
		//	cout << "one real root" << endl;
		D = sqrt(D);
		const double S = CubeRootFn(R + D);
		const double T = CubeRootFn(R - D);
		Rd = S + T;		
		
		if (Rd < 0) 
		{
			Rd = sqrt ( -1 / (3 * kappa1) );			
			stringstream strStream;
			strStream << "Undistorted image point to distorted image point mapping limited by maximum barrel distortion radius. Rd:";
			strStream <<  Rd << endl;
			strStream << "kappa1 =" << kappa1 << endl;
			strStream << "pUndistorted = [" << pUnistorted.X() << "; ";
			strStream << pUnistorted.Y() << "]";	
			string errMssg = strStream.str();			
			//throw errMssg;
		}   
		
		
	}
	else 
	{			/* three real roots */
		//cout << "three real roots" << endl;
		D = sqrt(-D);
		const double S = CubeRootFn( sqrt(R*R+D*D) );
		const double T = atan2 (D, R) / 3;		
		//SINCOS (T, sinT, cosT);
		/* the larger positive root is    2*S*cos(T)                   */
		/* the smaller positive root is   -S*cos(T) + SQRT(3)*S*sin(T) */
		/* the negative root is           -S*cos(T) - SQRT(3)*S*sin(T) */		
		//		Rd = -S*cos(T) - SQRT(3)*S*sin(T);	// negative root
		//	Rd = 2*S*cos(T);	// the larger positive root  
		
		Rd = -S * cos(T) + SQRT3 * S * sin(T);	/* use the smaller positive root */
	}	
	const double lambda = Rd / Ru;	
	
	pDistorted = Vector2d( pUnistorted.X() * lambda,  pUnistorted.Y() * lambda );	
	
}


double VcalCamera::CubeRootFn(const double x) const      
{    
	if (x == 0)
		return 0;
	else if (x > 0)
		return (pow (x, (double) 1.0 / 3.0));
	else
		return (-pow (-x, (double) 1.0 / 3.0));
} 

