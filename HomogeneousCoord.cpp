#include "HomogeneousCoord.h"
#include "helper.h"
#include "Matrix4x4.h"
#include <iostream>
#include <iomanip>


double HomogeneousCoord::DistTo(const HomogeneousCoord& v)
{
	return sqrt( 
		Sqr( GetX() - v.GetX() ) + 
		Sqr( GetY() - v.GetY() ) + 
		Sqr( GetZ() - v.GetZ() )
		);
}

void HomogeneousCoord::Print(const  char * str) const
{
	printf("%s=[%20.20f;%20.20f;%20.20f];\n",str,coord[0],coord[1],coord[2]);
}
void HomogeneousCoord::Print() const
{
	printf("vec=[%20.20f;%20.20f;%20.20f;%20.20f];\n",coord[0],coord[1],coord[2],coord[3]);
}

void HomogeneousCoord::PrintPlot() const
{
	printf("plot(%f,%f,'ob');fgcf;\n",coord[0],coord[1] );
}


HomogeneousCoord::HomogeneousCoord()
{
}

/*/
ostream& operator<< (ostream& theStream, const HomogeneousCoord& p)
{
	theStream << setiosflags(ios::showpoint | ios::fixed)
             << setprecision(20)           
             << setiosflags(ios::right); //  << setfill('*')
		
			theStream << "[";
			theStream << setw(25) << p.GetX() << ";";
			theStream << setw(25) << p.GetY() << ";";
			theStream << setw(25) << p.GetZ() << ";";
			theStream << setw(25) << p.GetW() << "]";
				
	
	return theStream;
}/*/


bool HomogeneousCoord::WithinDistance(const HomogeneousCoord v,
									  const double dist) const
{
	
	double vSqDist = Sqr(GetX() - v.GetX() ) 
		+ Sqr(GetY() - v.GetY() )
		+ Sqr(GetZ() - v.GetZ() );

	 
	if ( vSqDist < Sqr(dist) )
	{
		
//		cout << sqrt(vSqDist) << endl;
//		if ( sqrt(vSqDist) > glbBiggest )
//			glbBiggest = sqrt(vSqDist);

		return true;
	}
	else
	{
//		if ( sqrt(vSqDist) < glbSmallest )
//			glbSmallest = sqrt(vSqDist);

		return false;
	}

}

HomogeneousCoord::HomogeneousCoord(const double x,
								   const double y,
								   const double z,
								   const double w)
{
	coord[0]=x;
	coord[1]=y;
	coord[2]=z;
	coord[3]=w;
}

void HomogeneousCoord::Set(const double x, const double y)
{
	coord[0] = x;
	coord[1] = y;
	coord[2] = 1;
	coord[3] = 1;
}

void HomogeneousCoord::Set(const double x, const double y, const double z)
{
	coord[0] = x;
	coord[1] = y;
	coord[2] = z;
	coord[3] = 1;
}

HomogeneousCoord Cross(const HomogeneousCoord& a, const HomogeneousCoord& b)
{
	HomogeneousCoord result;
	
	result.coord[0] = a.coord[1] * b.coord[2] - a.coord[2] * b.coord[1];
	result.coord[1] = a.coord[2] * b.coord[0] - a.coord[0] * b.coord[2];
	result.coord[2] = a.coord[0] * b.coord[1] - a.coord[1] * b.coord[0];
	result.coord[3] = 1.0;
	
	return result;
}

double Dot(const HomogeneousCoord& a, const HomogeneousCoord& b)
{	
	return a.coord[0]*b.coord[0] + a.coord[1]*b.coord[1] + a.coord[2]*b.coord[2];
}


HomogeneousCoord FormUnitSurfNorm(
										   const HomogeneousCoord& v1,
										   const HomogeneousCoord& v2,
										   const HomogeneousCoord& v3) 
{
	HomogeneousCoord unitSurfNorm = Cross(v1.Minus(v2),v1.Minus(v3));
	double magnitude = sqrt(Sqr(unitSurfNorm.coord[0])+
		Sqr(unitSurfNorm.coord[1])+Sqr(unitSurfNorm.coord[2]));

	unitSurfNorm.coord[0] = unitSurfNorm.coord[0]/magnitude;
	unitSurfNorm.coord[1] = unitSurfNorm.coord[1]/magnitude;
	unitSurfNorm.coord[2] = unitSurfNorm.coord[2]/magnitude;

	return unitSurfNorm;
}


void HomogeneousCoord::ProjectOntoImagePlane()
{
	coord[0]=coord[0]/(coord[2]);
	coord[1]=coord[1]/(coord[2]);
	coord[2]=1.0;
	coord[3]=1.0;
}


HomogeneousCoord operator* (const double c, const HomogeneousCoord& v) 
{
	HomogeneousCoord result;
	
	result.coord[0]=c*v.coord[0];
	result.coord[1]=c*v.coord[1];
	result.coord[2]=c*v.coord[2];
	result.coord[3]=1;
	
	return result;
} 

HomogeneousCoord operator+ (const HomogeneousCoord& a, const HomogeneousCoord& b) 
{
	HomogeneousCoord result(a.GetX() + b.GetX(), a.GetY() + b.GetY(), a.GetZ() + b.GetZ(), 1);
	
	
	return result;
}

HomogeneousCoord operator* (const Matrix4x4& T, const HomogeneousCoord& v) 
{
	HomogeneousCoord result;
	
	result.coord[0]=T.element[0][0]*v.coord[0]+T.element[0][1]*v.coord[1]+T.element[0][2]*v.coord[2]+T.element[0][3]*v.coord[3];
	result.coord[1]=T.element[1][0]*v.coord[0]+T.element[1][1]*v.coord[1]+T.element[1][2]*v.coord[2]+T.element[1][3]*v.coord[3];
	result.coord[2]=T.element[2][0]*v.coord[0]+T.element[2][1]*v.coord[1]+T.element[2][2]*v.coord[2]+T.element[2][3]*v.coord[3];
	result.coord[3]=T.element[3][0]*v.coord[0]+T.element[3][1]*v.coord[1]+T.element[3][2]*v.coord[2]+T.element[3][3]*v.coord[3];
	
	return result;
} 


/*/
HomogeneousCoord operator- (const HomogeneousCoord& A, const HomogeneousCoord& B)
{
	HomogeneousCoord result;
	
	result.coord[0]=A.coord[0]-B.coord[0];
	result.coord[1]=A.coord[1]-B.coord[1];
	result.coord[2]=A.coord[2]-B.coord[2];
	result.coord[3]=1.0;
	
	return result;
} 
/*/


void HomogeneousCoord::Normalise()
{
	//double len = sqrt( Sqr(coord[0]) + Sqr(coord[1]) + Sqr(coord[2]) );
	double len = Norm3D();
	coord[0] /= len;
	coord[1] /= len;
	coord[2] /= len;
	coord[3] = 1;
}


HomogeneousCoord HomogeneousCoord::Minus (const HomogeneousCoord& B) const
{
	HomogeneousCoord result;
	
	result.coord[0]=coord[0]-B.coord[0];
	result.coord[1]=coord[1]-B.coord[1];
	result.coord[2]=coord[2]-B.coord[2];
	result.coord[3]=1.0;
	
	return result;
} 





ostream& operator<< (ostream& theStream, const HomogeneousCoord& v)
{
	theStream << setiosflags(ios::showpoint | ios::fixed)
		<< setprecision(10)           
		<< setiosflags(ios::right); //  << setfill('*')
	
		
		theStream << "[";
		
		theStream << setw(15) << v.GetX() << "; ";			
		theStream << setw(15) << v.GetY() << "; ";			
		theStream << setw(15) << v.GetZ() << "; ";			
		theStream << setw(15) << v.GetW();			

		theStream << "]";
				
	
	return theStream;
}
