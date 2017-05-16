// Geometric Tools
// http://www.geometrictools.com
// Copyright (c) 1998-2005.  All Rights Reserved
//
// The Wild Magic Library (WM3) source code is supplied under the terms of
// the license agreement
//     http://www.geometrictools.com/License/WildMagic3License.pdf
// and may not be copied or disclosed except in accordance with the terms
// of that agreement.

#ifndef WM3QUERY2_H
#define WM3QUERY2_H

#include "Wm3Vector2.h"

namespace Wm3
{

template <class Real>
class Query2
{
public:
    // The base class handles floating-point queries.
    Query2 (int iVQuantity, const Vector2<Real>* akVertex);
    virtual ~Query2 ();

    // member access
    int GetQuantity () const;
    const Vector2<Real>* GetVertices () const;

    // Queries about the relation of a point to various geometric objects.

    // returns
    //   +1, on right of line
    //   -1, on left of line
    //    0, on the line
    virtual int ToLine (int i, int iV0, int iV1) const;
    virtual int ToLine (const Vector2<Real>& rkP, int iV0, int iV1) const;

    // returns
    //   +1, outside triangle
    //   -1, inside triangle
    //    0, on triangle
    virtual int ToTriangle (int i, int iV0, int iV1, int iV2) const;
    virtual int ToTriangle (const Vector2<Real>& rkP, int iV0, int iV1,
        int iV2) const;

    // returns
    //   +1, outside circumcircle of triangle
    //   -1, inside circumcircle of triangle
    //    0, on circumcircle of triangle
    virtual int ToCircumcircle (int i, int iV0, int iV1, int iV2) const;
    virtual int ToCircumcircle (const Vector2<Real>& rkP, int iV0, int iV1,
        int iV2) const;

protected:
    // input points
    int m_iVQuantity;
    const Vector2<Real>* m_akVertex;

private:
    static Real Dot (Real fX0, Real fY0, Real fX1, Real fY1);

    static Real Det2 (Real fX0, Real fY0, Real fX1, Real fY1);

    static Real Det3 (Real iX0, Real iY0, Real iZ0, Real iX1, Real iY1,
        Real iZ1, Real iX2, Real iY2, Real iZ2);
};

#include "Wm3Query2.inl"

typedef Query2<float> Query2f;
typedef Query2<double> Query2d;

}

#endif

