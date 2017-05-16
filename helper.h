#ifndef HELPER_H
#define HELPER_H

#ifdef  _MSC_VER
// disable warning C4786: symbol greater than 255 character,
// okay to ignore this warning
#pragma warning(disable: 4786)
#endif  /* _MSC_VER */

#include <math.h>
#include <string>
using namespace std;

//this is so that the  a mex file can be easily compiled using MexWizard in the MSVC IDE
#ifdef MATLAB_MEX_FILE
#define INCLUDEMEX 1
#endif

#ifdef INCLUDEMEX
#include "mex.h"
#endif


#ifdef USE_GMPVH_ASSERTIONS
#define pvhAssert(e,s) {if ( ! (bool)(e) ) throw((std::string)(s));}
#else
	#ifdef NO_GMPVH_ASSERTIONS
		#define pvhAssert(e,s) {;}
	#else
		#ifdef NDEBUG
			#define pvhAssert(e,s) {;}
		#else
			#define pvhAssert(e,s) {if ( ! (bool)(e) ) throw((std::string)(s));}
		#endif
	#endif
#endif


const double PI = 4.0 * atan(1.0);
const double SLOPE30DEG = tan(30*PI/180); //the gradient (y/x) of a 30 degree line
const double PVH_EPSILON = 0.000001;

//to square x use Sqr(x)
template <class T>
inline T Sqr(const T& a) {return a*a;}

template <class T>
inline T Abs(const T& a) {return (a<0) ? -a : a;}


template <class T>
void Swap(T& a, T& b) 
{	
	T temp = a;
	a = b;
	b = temp;
}

std::string NextFile(const std::string baseName, const int digits, const std::string extension);


#endif