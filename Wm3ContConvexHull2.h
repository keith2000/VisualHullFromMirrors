// Geometric Tools
// http://www.geometrictools.com
// Copyright (c) 1998-2005.  All Rights Reserved
//
// The Wild Magic Library (WM3) source code is supplied under the terms of
// the license agreement
//     http://www.geometrictools.com/License/WildMagic3License.pdf
// and may not be copied or disclosed except in accordance with the terms
// of that agreement.

#ifndef WM3CONTCONVEXHULL2_H
#define WM3CONTCONVEXHULL2_H

#include "Wm3ContHullEdge2.h"
#include "Wm3Query2.h"

namespace Wm3
{

template <class Real>
class WM3_ITEM ConvexHull2
{
public:
    // Construction and destruction.  The caller is responsible for deleting
    // the akVertex array.
    ConvexHull2 (int iVQuantity, const Vector2<Real>* akVertex);
    ~ConvexHull2 ();

    // Compute the hull using an incremental method. The Boolean return
    // value is 'true' if the construction was successful.  If the value is
    // 'false', then a release-build run failed and a bug report should be
    // filed.
    bool DoIncremental (Real fEpsilon = (Real)0.001);

    // The dimension of the hull is in {0,1,2}.  The index array has
    // GetQuantity() elements. The possibilities are
    //     dim = 0, quantity = 1, indices = {0}
    //     dim = 1, quantity = 2, indices = {i[0],i[1]}
    //     dim = 2, quantity >= 3, indices = {i[0],...,i[quantity-1]}
    int GetDimension () const;
    int GetQuantity () const;
    int* GetIndices (bool bTakeOwnership = false);

    // If the hull dimension is 1, the hull is linear and lives on the line
    // with these parameters.
    const Vector2<Real>& GetLineOrigin () const;
    const Vector2<Real>& GetLineDirection () const;

private:
    bool Update (HullEdge2<Real>*& rpkHull, int i);

    // the input points
    int m_iVQuantity;
    const Vector2<Real>* m_akVertex;
    Vector2<Real>* m_akSVertex;

    // for robust queries
    Query2<Real>* m_pkQuery;

    // for a linear hull only
    Vector2<Real> m_kLineOrigin, m_kLineDirection;

    // the hull
    int m_iHDim, m_iHQuantity;
    int* m_aiHIndex;
};

typedef ConvexHull2<float> ConvexHull2f;
typedef ConvexHull2<double> ConvexHull2d;

}

#endif


