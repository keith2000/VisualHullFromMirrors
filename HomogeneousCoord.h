#ifndef HOMOGENEOUSCOORD_H
#define HOMOGENEOUSCOORD_H

#include "helper.h"

class Matrix4x4;
class PolygonSet;
class ClosedContour;
class VisualHull;

class HomogeneousCoord
{
public:
	friend HomogeneousCoord operator* (const Matrix4x4 &T, const HomogeneousCoord &v);
	friend HomogeneousCoord operator* (const double c, const HomogeneousCoord& v);
	friend HomogeneousCoord operator+ (const HomogeneousCoord& a, const HomogeneousCoord& b);
	//friend HomogeneousCoord operator- (const HomogeneousCoord& a, const HomogeneousCoord& b);
	friend PolygonSet;
	friend ClosedContour;
	friend VisualHull;
	friend HomogeneousCoord FormUnitSurfNorm(const HomogeneousCoord& v1,
	const HomogeneousCoord& v2, const HomogeneousCoord& v3);
	friend double Dot(const HomogeneousCoord& a, const HomogeneousCoord& b);
	friend HomogeneousCoord Cross(const HomogeneousCoord& a, const HomogeneousCoord& b);
		
	HomogeneousCoord();
	HomogeneousCoord(const double x, const double y, const double z, const double w);	
	HomogeneousCoord Minus(const HomogeneousCoord& B) const;

	double DistTo(const HomogeneousCoord& v);

	double Norm3D() const {return sqrt(Sqr(coord[0])+Sqr(coord[1])+Sqr(coord[2]) );}
	double Norm2D() const {return sqrt(Sqr(coord[0])+Sqr(coord[1]) );}

	void ProjectOntoImagePlane();

	inline double GetX() const {return coord[0];}
	inline double GetY() const {return coord[1];}
	inline double GetZ() const {return coord[2];}
	inline double GetW() const {return coord[3];}

	void Set(const double x, const double y);	
	void Set(const double x, const double y, const double z);	
	void Print() const;
	void Print(const char * str) const;
	void PrintPlot() const;

	void Normalise();

	bool WithinDistance(const HomogeneousCoord v,
									  const double dist) const;	
private:
	double coord[4];
};


std::ostream& operator<< (std::ostream& theStream, const HomogeneousCoord& v);

#endif