#ifndef MATRIX4X4_H
#define MATRIX4X4_H

#include "helper.h"
#include "HomogeneousCoord.h"


struct CvMat;
struct mxArray_tag;

class Matrix4x4
{
public:
	friend HomogeneousCoord operator* (const Matrix4x4& T, const HomogeneousCoord& v);
	friend Matrix4x4 operator* (const Matrix4x4& M1, const Matrix4x4& M2);

	virtual ~Matrix4x4();
	Matrix4x4();
	Matrix4x4(double * data);
	
	Matrix4x4(const mxArray_tag* M);

	Matrix4x4(double qX, double qY, double qZ, double qW);

	Matrix4x4(
		double e00, double e01, double e02, double e03,
		double e10, double e11, double e12, double e13,
		double e20, double e21, double e22, double e23,
		double e30, double e31, double e32, double e33
		);


	Matrix4x4(const HomogeneousCoord& r0,
		const HomogeneousCoord& r1,
		const HomogeneousCoord& r2,
		const HomogeneousCoord& t);

	Matrix4x4(const Matrix4x4& M,
		const HomogeneousCoord& h1,
		const HomogeneousCoord& h2,
		const HomogeneousCoord& P0); //Form 4x4 Heikilla-style matrix, H

	inline double GetElement(const unsigned r,const unsigned c) const{return element[r][c];}
	inline void SetElement(unsigned r, unsigned c, double val) {element[r][c]=val;}
	
	inline double& operator() (unsigned r, unsigned c) { return element[r][c]; }
    inline double  operator() (unsigned r, unsigned c) const 	{ return element[r][c]; }
	


	void Form(const CvMat * M);

	void Print() const;

	void Print(const  char * str) const;
	Matrix4x4 Inverse() const;
	Matrix4x4 RBTInverse() const;
	Matrix4x4 CramerInverse() const;
	void UnitQuaternion( double * q ) const;
	void UniformRandomRotation();

	void AlignVectors(const HomogeneousCoord& vA, const HomogeneousCoord& vB);
	//sets up a  rotation matrix to align vB with vA i.e. vA==R*vB


	void FormFromMatlabArray(const mxArray_tag* M);

		
private:
	double element[4][4];
};


std::ostream& operator<< (std::ostream& theStream, const Matrix4x4& M);

#endif