#include "helper.h"
#include "Matrix4x4.h"
//#include <stdio.h>
//#include <stdlib.h>

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <iomanip>
using namespace std;




void InvertMatrix4x4(Matrix4x4 src,Matrix4x4& dest);

Matrix4x4 Matrix4x4::Inverse() const
{
	Matrix4x4 src(*this);
	Matrix4x4 dest;
	InvertMatrix4x4(src,dest);
	return dest;
}



Matrix4x4 Matrix4x4::CramerInverse() const
{
	Matrix4x4 dest;

	//code derived from:
	//Streaming SIMD Extensions -
	//Inverse of 4x4 Matrix

	double srcTrans[16]; /* array of transpose source matrix */
	
	for ( int i = 0; i < 4; i++) 
	{
		srcTrans[i]     = GetElement(i,0);
		srcTrans[i + 4] = GetElement(i,1);
		srcTrans[i + 8] = GetElement(i,2);
		srcTrans[i + 12]= GetElement(i,3);
	}

	double tmp[12];

	tmp[ 0] = srcTrans[10] * srcTrans[15];
	tmp[ 1] = srcTrans[11] * srcTrans[14];
	tmp[ 2] = srcTrans[ 9] * srcTrans[15];
	tmp[ 3] = srcTrans[11] * srcTrans[13];
	tmp[ 4] = srcTrans[ 9] * srcTrans[14];
	tmp[ 5] = srcTrans[10] * srcTrans[13];
	tmp[ 6] = srcTrans[ 8] * srcTrans[15];
	tmp[ 7] = srcTrans[11] * srcTrans[12];
	tmp[ 8] = srcTrans[ 8] * srcTrans[14];
	tmp[ 9] = srcTrans[10] * srcTrans[12];
	tmp[10] = srcTrans[ 8] * srcTrans[13];
	tmp[11] = srcTrans[ 9] * srcTrans[12];

	/* calculate first 8 elements (cofactors) */
	double dst[16];
	dst[0] = tmp[0]*srcTrans[5] + tmp[3]*srcTrans[6] + tmp[4]*srcTrans[7];
	dst[0] -= tmp[1]*srcTrans[5] + tmp[2]*srcTrans[6] + tmp[5]*srcTrans[7];
	dst[1] = tmp[1]*srcTrans[4] + tmp[6]*srcTrans[6] + tmp[9]*srcTrans[7];
	dst[1] -= tmp[0]*srcTrans[4] + tmp[7]*srcTrans[6] + tmp[8]*srcTrans[7];
	dst[2] = tmp[2]*srcTrans[4] + tmp[7]*srcTrans[5] + tmp[10]*srcTrans[7];
	dst[2] -= tmp[3]*srcTrans[4] + tmp[6]*srcTrans[5] + tmp[11]*srcTrans[7];
	dst[3] = tmp[5]*srcTrans[4] + tmp[8]*srcTrans[5] + tmp[11]*srcTrans[6];
	dst[3] -= tmp[4]*srcTrans[4] + tmp[9]*srcTrans[5] + tmp[10]*srcTrans[6];
	dst[4] = tmp[1]*srcTrans[1] + tmp[2]*srcTrans[2] + tmp[5]*srcTrans[3];
	dst[4] -= tmp[0]*srcTrans[1] + tmp[3]*srcTrans[2] + tmp[4]*srcTrans[3];
	dst[5] = tmp[0]*srcTrans[0] + tmp[7]*srcTrans[2] + tmp[8]*srcTrans[3];
	dst[5] -= tmp[1]*srcTrans[0] + tmp[6]*srcTrans[2] + tmp[9]*srcTrans[3];
	dst[6] = tmp[3]*srcTrans[0] + tmp[6]*srcTrans[1] + tmp[11]*srcTrans[3];
	dst[6] -= tmp[2]*srcTrans[0] + tmp[7]*srcTrans[1] + tmp[10]*srcTrans[3];
	dst[7] = tmp[4]*srcTrans[0] + tmp[9]*srcTrans[1] + tmp[10]*srcTrans[2];
	dst[7] -= tmp[5]*srcTrans[0] + tmp[8]*srcTrans[1] + tmp[11]*srcTrans[2];

	/* calculate pairs for second 8 elements (cofactors) */
	tmp[0] = srcTrans[2]*srcTrans[7];
	tmp[1] = srcTrans[3]*srcTrans[6];
	tmp[2] = srcTrans[1]*srcTrans[7];
	tmp[3] = srcTrans[3]*srcTrans[5];
	tmp[4] = srcTrans[1]*srcTrans[6];
	tmp[5] = srcTrans[2]*srcTrans[5];
	tmp[6] = srcTrans[0]*srcTrans[7];
	tmp[7] = srcTrans[3]*srcTrans[4];
	tmp[8] = srcTrans[0]*srcTrans[6];
	tmp[9] = srcTrans[2]*srcTrans[4];
	tmp[10] = srcTrans[0]*srcTrans[5];
	tmp[11] = srcTrans[1]*srcTrans[4];

	/* calculate second 8 elements (cofactors) */
	dst[8] = tmp[0]*srcTrans[13] + tmp[3]*srcTrans[14] + tmp[4]*srcTrans[15];
	dst[8] -= tmp[1]*srcTrans[13] + tmp[2]*srcTrans[14] + tmp[5]*srcTrans[15];
	dst[9] = tmp[1]*srcTrans[12] + tmp[6]*srcTrans[14] + tmp[9]*srcTrans[15];
	dst[9] -= tmp[0]*srcTrans[12] + tmp[7]*srcTrans[14] + tmp[8]*srcTrans[15];
	dst[10] = tmp[2]*srcTrans[12] + tmp[7]*srcTrans[13] + tmp[10]*srcTrans[15];
	dst[10]-= tmp[3]*srcTrans[12] + tmp[6]*srcTrans[13] + tmp[11]*srcTrans[15];
	dst[11] = tmp[5]*srcTrans[12] + tmp[8]*srcTrans[13] + tmp[11]*srcTrans[14];
	dst[11]-= tmp[4]*srcTrans[12] + tmp[9]*srcTrans[13] + tmp[10]*srcTrans[14];
	dst[12] = tmp[2]*srcTrans[10] + tmp[5]*srcTrans[11] + tmp[1]*srcTrans[9];
	dst[12]-= tmp[4]*srcTrans[11] + tmp[0]*srcTrans[9] + tmp[3]*srcTrans[10];
	dst[13] = tmp[8]*srcTrans[11] + tmp[0]*srcTrans[8] + tmp[7]*srcTrans[10];
	dst[13]-= tmp[6]*srcTrans[10] + tmp[9]*srcTrans[11] + tmp[1]*srcTrans[8];
	dst[14] = tmp[6]*srcTrans[9] + tmp[11]*srcTrans[11] + tmp[3]*srcTrans[8];
	dst[14]-= tmp[10]*srcTrans[11] + tmp[2]*srcTrans[8] + tmp[7]*srcTrans[9];
	dst[15] = tmp[10]*srcTrans[10] + tmp[4]*srcTrans[8] + tmp[9]*srcTrans[9];
	dst[15]-= tmp[8]*srcTrans[9] + tmp[11]*srcTrans[10] + tmp[5]*srcTrans[8];


	/* calculate determinant */
	double det = srcTrans[0] * dst[0] +
		srcTrans[1] * dst[1] +
		srcTrans[2] * dst[2] +
		srcTrans[3] * dst[3];
	/* calculate matrix inverse */

	//cout << det << endl;
	//pvhAssert( Abs(det) > PVH_EPSILON, "div by number close to 0 in cramer inverse");
	//
	//det = 1e+000/det; //multiple by a million to prevent problems later divide by a million
	det = 1/det;

	for(int c=0;c<4;c++)
	{
		for(int r=0;r<4;r++)
		{
			//dest(r,c) = ( det * dst[r*4 + c] ) * 1e-000; //now multiply by one millionth
			//dest(r,c) = dst[r*4 + c] / det; //in this way we can handle smaller numbers!
			dest(r,c) = dst[r*4 + c] * det;
			// at the expense of 15 more divides
		}
	}

	return dest;
}


void Matrix4x4::UniformRandomRotation()
{
	//@incollection{arvo,
    //author={James Arvo},
    //title={Fast Random Rotation Matrices},
    //booktitle={Graphics Gems III},
    //publisher={Academic Press},
    //year= 1992,
    //editor={David Kirk},
    //pages={117--120}}

	double theta = 2 * PI * (double)rand()/(double)RAND_MAX;
	double phi = 2 * PI * (double)rand()/(double)RAND_MAX;
	double zz = (double)rand()/(double)RAND_MAX;


//	printf("theta: %f, phi: %f, zz: %f, RAND_MAX: %d\n",
//		theta*180/PI,phi*180/PI,zz,RAND_MAX);

	double v0 = cos(phi) * sqrt(zz);
	double v1 = sin(phi) * sqrt(zz);
	double v2 = sqrt(1-zz);

	double VVI[3][3];

	VVI[0][0] = 2 * v0 * v0 - 1;
	VVI[1][0] = 2 * v0 * v1 - 0;
	VVI[2][0] = 2 * v0 * v2 - 0;

	VVI[0][1] = 2 * v1 * v0 - 0;
	VVI[1][1] = 2 * v1 * v1 - 1;
	VVI[2][1] = 2 * v1 * v2 - 0;

	VVI[0][2] = 2 * v2 * v0 - 0;
	VVI[1][2] = 2 * v2 * v1 - 0;
	VVI[2][2] = 2 * v2 * v2 - 1;

	double cosTheta = cos(theta);
	double sinTheta = sin(theta);

	double R[3][3];

	R[0][0] = cosTheta;
	R[1][0] = -sinTheta;
	R[2][0] = 0;
	
	R[0][1] = sinTheta;
	R[1][1] = cosTheta;
	R[2][1] = 0;

	R[0][2] = 0;
	R[1][2] = 0;
	R[2][2] = 1;


	element[0][0] = VVI[0][0] * R[0][0] + VVI[0][1] * R[1][0] + VVI[0][2] * R[2][0];
	element[0][1] = VVI[0][0] * R[0][1] + VVI[0][1] * R[1][1] + VVI[0][2] * R[2][1];
	element[0][2] = VVI[0][0] * R[0][2] + VVI[0][1] * R[1][2] + VVI[0][2] * R[2][2];
	element[0][3] = 0;

	element[1][0] = VVI[1][0] * R[0][0] + VVI[1][1] * R[1][0] + VVI[1][2] * R[2][0];
	element[1][1] = VVI[1][0] * R[0][1] + VVI[1][1] * R[1][1] + VVI[1][2] * R[2][1];
	element[1][2] = VVI[1][0] * R[0][2] + VVI[1][1] * R[1][2] + VVI[1][2] * R[2][2];
	element[1][3] = 0;

	element[2][0] = VVI[2][0] * R[0][0] + VVI[2][1] * R[1][0] + VVI[2][2] * R[2][0];
	element[2][1] = VVI[2][0] * R[0][1] + VVI[2][1] * R[1][1] + VVI[2][2] * R[2][1];
	element[2][2] = VVI[2][0] * R[0][2] + VVI[2][1] * R[1][2] + VVI[2][2] * R[2][2];
	element[2][3] = 0;

	element[3][0] = 0;
	element[3][1] = 0;
	element[3][2] = 0;
	element[3][3] = 1;

}

Matrix4x4 Matrix4x4::RBTInverse() const
{
	//Invert 4x4 rigid body transform matrices
	//matrix is assumed to represent a rigid body transform
	Matrix4x4 dest;

	
	dest.SetElement(0,0,GetElement(0,0));
	dest.SetElement(0,1,GetElement(1,0));
	dest.SetElement(0,2,GetElement(2,0));
	dest.SetElement(0,3,\
		-GetElement(0,0) * GetElement(0,3)
		-GetElement(1,0) * GetElement(1,3)
		-GetElement(2,0) * GetElement(2,3)
		);

	dest.SetElement(1,0,GetElement(0,1));
	dest.SetElement(1,1,GetElement(1,1));
	dest.SetElement(1,2,GetElement(2,1));
	dest.SetElement(1,3,
		-GetElement(0,1) * GetElement(0,3)
		-GetElement(1,1) * GetElement(1,3)
		-GetElement(2,1) * GetElement(2,3)
		);

	dest.SetElement(2,0,GetElement(0,2));
	dest.SetElement(2,1,GetElement(1,2));
	dest.SetElement(2,2,GetElement(2,2));
	dest.SetElement(2,3,
		-GetElement(0,2) * GetElement(0,3)
		-GetElement(1,2) * GetElement(1,3)
		-GetElement(2,2) * GetElement(2,3)
		);

	dest.SetElement(3,0,0.0);
	dest.SetElement(3,1,0.0);
	dest.SetElement(3,2,0.0);
	dest.SetElement(3,3,1.0);

	return dest;
}


Matrix4x4::~Matrix4x4()
{
}

void Matrix4x4::AlignVectors(const HomogeneousCoord& vA, const HomogeneousCoord& vB)
{
	//This is based on the Rodrigues' formula

	HomogeneousCoord v1(vA);
	v1.Normalise();

	HomogeneousCoord v2(vB);
	v2.Normalise();

	double cosTheta = Dot(v1,v2);

    HomogeneousCoord w = Cross(v2,v1);
    
	double sinTheta = w.Norm3D(); //store the magnitude before it's set to one    
	w.Normalise();

	element[0][0] = cosTheta * 1 + sinTheta * 0.0000000 + (1-cosTheta) * w.GetX() * w.GetX();
	element[0][1] = cosTheta * 0 + sinTheta * -w.GetZ() + (1-cosTheta) * w.GetX() * w.GetY();
	element[0][2] = cosTheta * 0 + sinTheta * +w.GetY() + (1-cosTheta) * w.GetX() * w.GetZ();
	element[0][3] = 0;

	element[1][0] = cosTheta * 0 + sinTheta * +w.GetZ() + (1-cosTheta) * w.GetY() * w.GetX();
	element[1][1] = cosTheta * 1 + sinTheta * 0.0000000 + (1-cosTheta) * w.GetY() * w.GetY();
	element[1][2] = cosTheta * 0 + sinTheta * -w.GetX() + (1-cosTheta) * w.GetY() * w.GetZ();
	element[1][3] = 0;

	element[2][0] = cosTheta * 0 + sinTheta * -w.GetY() + (1-cosTheta) * w.GetZ() * w.GetX();
	element[2][1] = cosTheta * 0 + sinTheta * +w.GetX() + (1-cosTheta) * w.GetZ() * w.GetY();
	element[2][2] = cosTheta * 1 + sinTheta * 0.0000000 + (1-cosTheta) * w.GetZ() * w.GetZ();
	element[2][3] = 0;

	element[3][0] = 0;
	element[3][1] = 0;
	element[3][2] = 0;
	element[3][3] = 1;

}

Matrix4x4::Matrix4x4()
{
}


Matrix4x4::Matrix4x4(
					 double e00, double e01, double e02, double e03,
					 double e10, double e11, double e12, double e13,
					 double e20, double e21, double e22, double e23,
					 double e30, double e31, double e32, double e33
					 )
{
	element[0][0] = e00;
	element[1][0] = e10;
	element[2][0] = e20;
	element[3][0] = e30;
	
	element[0][1] = e01;
	element[1][1] = e11;
	element[2][1] = e21;
	element[3][1] = e31;

	element[0][2] = e02;
	element[1][2] = e12;
	element[2][2] = e22;
	element[3][2] = e32;
	
	element[0][3] = e03;
	element[1][3] = e13;
	element[2][3] = e23;
	element[3][3] = e33;
	
}

Matrix4x4::Matrix4x4(double qX, double qY, double qZ, double qW)
{

	double norm = sqrt( qX*qX + qY*qY + qZ*qZ + qW*qW );
	qX /= norm;
	qY /= norm;
	qZ /= norm;
	qW /= norm;


	element[0][0] = 1 -  2*qY*qY - 2*qZ*qZ;;
	element[1][0] = 2*qX*qY +2*qZ*qW;
	element[2][0] = 2*qX*qZ -2*qY*qW;
	element[3][0] = 0;

	element[0][1] = 2*qX*qY - 2*qZ*qW;
	element[1][1] = 1 - 2*qX*qX - 2*qZ*qZ;
	element[2][1] = 2*qY*qZ + 2*qX*qW;
	element[3][1] = 0;

	element[0][2] = 2*qX*qZ + 2*qY*qW;
	element[1][2] = 2*qY*qZ - 2*qX*qW;
	element[2][2] = 1 - 2*qX*qX - 2*qY*qY;
	element[3][2] = 0;

	element[0][3] = 0;
	element[1][3] = 0;
	element[2][3] = 0;
	element[3][3] = 1;
}


Matrix4x4::Matrix4x4(const HomogeneousCoord& r0,
					 const HomogeneousCoord& r1,
					 const HomogeneousCoord& r2,
					 const HomogeneousCoord& t)
{
	element[0][0] = r0.GetX();
	element[1][0] = r0.GetY();
	element[2][0] = r0.GetZ();
	element[3][0] = 0;

	element[0][1] = r1.GetX();
	element[1][1] = r1.GetY();
	element[2][1] = r1.GetZ();
	element[3][1] = 0;

	element[0][2] = r2.GetX();
	element[1][2] = r2.GetY();
	element[2][2] = r2.GetZ();
	element[3][2] = 0;

	element[0][3] = t.GetX();
	element[1][3] = t.GetY();
	element[2][3] = t.GetZ();
	element[3][3] = 1;
}


Matrix4x4::Matrix4x4(double * data)
{
	for(int r=0; r<4 ; r++)
		for(int c=0; c<4 ; c++)
			element[r][c]=data[4*c+r];
}

Matrix4x4::Matrix4x4(const Matrix4x4& M,
					 const HomogeneousCoord& h1,
					 const HomogeneousCoord& h2,
					 const HomogeneousCoord& P0)
 //Form 4x4 Heikilla-style matrix, H
{
	element[0][0] = M(0,0) * h1.GetX() + M(0,1) * h1.GetY() + M(0,2) * h1.GetZ();
	element[1][0] = M(1,0) * h1.GetX() + M(1,1) * h1.GetY() + M(1,2) * h1.GetZ();
	element[2][0] = M(2,0) * h1.GetX() + M(2,1) * h1.GetY() + M(2,2) * h1.GetZ();
	element[3][0] = 0.0;

	element[0][1] = M(0,0) * h2.GetX() + M(0,1) * h2.GetY() + M(0,2) * h2.GetZ();
	element[1][1] = M(1,0) * h2.GetX() + M(1,1) * h2.GetY() + M(1,2) * h2.GetZ();
	element[2][1] = M(2,0) * h2.GetX() + M(2,1) * h2.GetY() + M(2,2) * h2.GetZ();
	element[3][1] = 0.0;

	element[0][2] = M(0,0) * P0.GetX() + M(0,1) * P0.GetY() + M(0,2) * P0.GetZ() + M(0,3);
	element[1][2] = M(1,0) * P0.GetX() + M(1,1) * P0.GetY() + M(1,2) * P0.GetZ() + M(1,3);
	element[2][2] = M(2,0) * P0.GetX() + M(2,1) * P0.GetY() + M(2,2) * P0.GetZ() + M(2,3);
	element[3][2] = 0.0;

	element[0][3] = 0.0;
	element[1][3] = 0.0;
	element[2][3] = 0.0;
	element[3][3] = 1.0;

}


void Matrix4x4::Print(const  char * str) const
{
	printf("%s=[",str);
	for(int c=0;c<4;c++)
	{
		printf("[%20.20f;%20.20f;%20.20f;%20.20f] ",element[0][c],element[1][c],element[2][c],element[3][c]);
		
	}	
	printf("];\n");
}

void Matrix4x4::Print() const
{
	printf("|%10.5f %10.5f %10.5f %10.5f|\n",element[0][0],element[0][1],element[0][2],element[0][3]);
	printf("|%10.5f %10.5f %10.5f %10.5f|\n",element[1][0],element[1][1],element[1][2],element[1][3]);
	printf("|%10.5f %10.5f %10.5f %10.5f|\n",element[2][0],element[2][1],element[2][2],element[2][3]);
	printf("|%10.5f %10.5f %10.5f %10.5f|\n",element[3][0],element[3][1],element[3][2],element[3][3]);			
	printf("M=[");
	for(int c=0;c<4;c++)
	{
		printf("...\n[%20.20f;%20.20f;...\n%20.20f;%20.20f] ",element[0][c],element[1][c],element[2][c],element[3][c]);
		
	}	
	printf("];\n");
}


void InvertMatrix4x4(Matrix4x4 src,Matrix4x4& dest)
{	
	const int n=4 ;
	int i,j,m ;
	double temp;
	
	for(i = 0; i < n ; i++)
		for(j = 0; j < n; j++) 
		{ 
			if(i == j) 
				dest(i,j)= 1 ; 
			else 
				dest(i,j)= 0 ; 
		}
//		printf("a");
		for( i = 0;i < n; i++)
			for( j = i+1;j < n;j++)
			{ 
			//	printf(" src(i,i): %f\n",src(i,i));
				pvhAssert(Abs(src(i,i)) > PVH_EPSILON,"matrix inversion -- div by zero");
			//	printf("i:%d\n",i);
				temp= - src(j,i)/src(i,i);
			//	printf("d");
				
				
				for(m = 0 ; m < n ; m++)
				{ 
					src(j,m)+=temp*src(i,m) ;
					dest(j,m)+=temp*dest(i,m) ;
				}
			}
//			printf("b");
			for( i = n-1; i >= 0 ; i--)
			{  
				for(j = i-1; j>=0; j--)
				{ 
					pvhAssert(Abs(src(i,i)) > PVH_EPSILON,"matrix inversion -- div by zero");
					
					temp= - src(j,i)/src(i,i) ;
					
					for( m=0 ; m < n ; m++)
					{ 
						src(j,m) += temp * src(i,m) ;
						dest(j,m) += temp * dest(i,m) ;
					}
				}
				
				pvhAssert(Abs(src(i,i)) > PVH_EPSILON,"matrix inversion -- div by zero");
				temp=1/src(i,i) ;
				
				for( m = 0 ; m < n; m++ )
				{
					dest(i,m) *= temp ;
					src(i,m) *= temp ;
				}
			}
}





ostream& operator<< (ostream& theStream, const Matrix4x4& M)
{
	theStream << setiosflags(ios::showpoint | ios::fixed)
		<< setprecision(20)           
		<< setiosflags(ios::right); //  << setfill('*')
	
	theStream << "[";
	for (int r = 0; r < 4; r++)
	{
		if ( r > 0)
			theStream << ";";
		
		theStream << "[";
		for (int c = 0; c < 4; c++)
		{
			if ( c > 0)
				theStream << ",";						
			theStream << setw(25) << M(r,c);			
			
		}
		theStream << "]";
	}
	theStream << "]";
				
	
	return theStream;
}


Matrix4x4 operator* (const Matrix4x4& M1, const Matrix4x4& M2) 
{
	Matrix4x4 result;	

	result.element[0][0] = M1.element[0][0] * M2.element[0][0] + M1.element[0][1] * M2.element[1][0] + M1.element[0][2] * M2.element[2][0] + M1.element[0][3] * M2.element[3][0];
	result.element[1][0] = M1.element[1][0] * M2.element[0][0] + M1.element[1][1] * M2.element[1][0] + M1.element[1][2] * M2.element[2][0] + M1.element[1][3] * M2.element[3][0];
	result.element[2][0] = M1.element[2][0] * M2.element[0][0] + M1.element[2][1] * M2.element[1][0] + M1.element[2][2] * M2.element[2][0] + M1.element[2][3] * M2.element[3][0];
	result.element[3][0] = M1.element[3][0] * M2.element[0][0] + M1.element[3][1] * M2.element[1][0] + M1.element[3][2] * M2.element[2][0] + M1.element[3][3] * M2.element[3][0];
	
	result.element[0][1] = M1.element[0][0] * M2.element[0][1] + M1.element[0][1] * M2.element[1][1] + M1.element[0][2] * M2.element[2][1] + M1.element[0][3] * M2.element[3][1];
	result.element[1][1] = M1.element[1][0] * M2.element[0][1] + M1.element[1][1] * M2.element[1][1] + M1.element[1][2] * M2.element[2][1] + M1.element[1][3] * M2.element[3][1];
	result.element[2][1] = M1.element[2][0] * M2.element[0][1] + M1.element[2][1] * M2.element[1][1] + M1.element[2][2] * M2.element[2][1] + M1.element[2][3] * M2.element[3][1];
	result.element[3][1] = M1.element[3][0] * M2.element[0][1] + M1.element[3][1] * M2.element[1][1] + M1.element[3][2] * M2.element[2][1] + M1.element[3][3] * M2.element[3][1];
	
	result.element[0][2] = M1.element[0][0] * M2.element[0][2] + M1.element[0][1] * M2.element[1][2] + M1.element[0][2] * M2.element[2][2] + M1.element[0][3] * M2.element[3][2];
	result.element[1][2] = M1.element[1][0] * M2.element[0][2] + M1.element[1][1] * M2.element[1][2] + M1.element[1][2] * M2.element[2][2] + M1.element[1][3] * M2.element[3][2];
	result.element[2][2] = M1.element[2][0] * M2.element[0][2] + M1.element[2][1] * M2.element[1][2] + M1.element[2][2] * M2.element[2][2] + M1.element[2][3] * M2.element[3][2];
	result.element[3][2] = M1.element[3][0] * M2.element[0][2] + M1.element[3][1] * M2.element[1][2] + M1.element[3][2] * M2.element[2][2] + M1.element[3][3] * M2.element[3][2];
	
	result.element[0][3] = M1.element[0][0] * M2.element[0][3] + M1.element[0][1] * M2.element[1][3] + M1.element[0][2] * M2.element[2][3] + M1.element[0][3] * M2.element[3][3];
	result.element[1][3] = M1.element[1][0] * M2.element[0][3] + M1.element[1][1] * M2.element[1][3] + M1.element[1][2] * M2.element[2][3] + M1.element[1][3] * M2.element[3][3];
	result.element[2][3] = M1.element[2][0] * M2.element[0][3] + M1.element[2][1] * M2.element[1][3] + M1.element[2][2] * M2.element[2][3] + M1.element[2][3] * M2.element[3][3];
	result.element[3][3] = M1.element[3][0] * M2.element[0][3] + M1.element[3][1] * M2.element[1][3] + M1.element[3][2] * M2.element[2][3] + M1.element[3][3] * M2.element[3][3];
	
	
	return result;
} 

void Matrix4x4::UnitQuaternion( double * q) const
{
	double T = element[0][0] + element[1][1] + element[2][2] + element[3][3];
	
	double R32 = element[2][1];
	double R23 = element[1][2];
	double R31 = element[2][0];
	double R13 = element[0][2];
	double R21 = element[1][0];
	double R12 = element[0][1];
	double S;
		
	if ( T > 0 )
	{
		 S = 0.5 / sqrt(T);        
		q[0] = ( R32 - R23 ) * S;    
		q[1] = ( R13 - R31 ) * S;    
		q[2] = ( R21 - R12 ) * S;    
		q[3] = 0.25 / S;        
	}
	else    
	{
		
		double R11 = element[0][0];
		double R22 = element[1][1];
		double R33 = element[2][2];
		
		int ind;
		ind = (R11 > R22) ? 0 : 1;
		ind = (element[ind][ind] > R33) ? ind : 2;
		
		
		switch( ind ) 
		{
		case 0:
			 S  = sqrt( 1.0 + R11 - R22 - R33 ) * 2;
			q[0] = 0.5 / S;
			q[1] = (R12 + R21 ) / S;
			q[2] = (R13 + R31 ) / S;
			q[3] = (R23 + R32 ) / S;        
			break;
		case 1 :
			 S  = sqrt( 1.0 + R22 - R11 - R33 ) * 2;        
			q[0] = (R12 + R21 ) / S;
			q[1] = 0.5 / S;
			q[2] = (R23 + R32 ) / S;
			q[3] = (R13 + R31 ) / S;			
			break;
		case 2 :
			 S  = sqrt( 1.0 + R33 - R11 - R22 ) * 2;        
			q[0] = (R13 + R31 ) / S;
			q[1] = (R23 + R32 ) / S;
			q[2] = 0.5 / S;
			q[3] = (R12 + R21 ) / S;
			
			break;
		}
	}
}