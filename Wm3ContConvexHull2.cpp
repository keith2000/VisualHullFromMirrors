// Geometric Tools
// http://www.geometrictools.com
// Copyright (c) 1998-2005.  All Rights Reserved
//
// The Wild Magic Library (WM3) source code is supplied under the terms of
// the license agreement
//     http://www.geometrictools.com/License/WildMagic3License.pdf
// and may not be copied or disclosed except in accordance with the terms
// of that agreement.

#include "Wm3ContConvexHull2.h"
#include "Wm3Mapper2.h"

// The fastest choice is Query2Int64.  The others may be swapped in for
// testing by enabling exactly one of the following defines.
#define USE_QUERY2INT64
//#define USE_QUERY2TINTEGER
//#define USE_QUERY2TRATIONAL

#ifdef USE_QUERY2INT64
#include "Wm3Query2Int64.h"
#endif

#ifdef USE_QUERY2TINTEGER
#include "Wm3Query2TInteger.h"
#endif

#ifdef USE_QUERY2TRATIONAL
#include "Wm3Query2TRational.h"
#endif

namespace Wm3
{

//----------------------------------------------------------------------------
template <class Real>
ConvexHull2<Real>::ConvexHull2 (int iVQuantity, const Vector2<Real>* akVertex)
    :
    m_kLineOrigin(Vector2<Real>::ZERO),
    m_kLineDirection(Vector2<Real>::ZERO)
{
    assert( iVQuantity > 0 && akVertex );
    m_iVQuantity = iVQuantity;
    m_akVertex = akVertex;
    m_akSVertex = 0;
    m_pkQuery = 0;
    m_iHDim = 0;
    m_iHQuantity = 0;
    m_aiHIndex = 0;
}
//----------------------------------------------------------------------------
template <class Real>
ConvexHull2<Real>::~ConvexHull2 ()
{
    assert( m_pkQuery == 0 );
    delete[] m_aiHIndex;

#ifndef USE_QUERY2TRATIONAL
    delete[] m_akSVertex;
#endif
}
//----------------------------------------------------------------------------
template <class Real>
bool ConvexHull2<Real>::DoIncremental (Real fEpsilon)
{
    if ( fEpsilon < (Real)0.0 )
        fEpsilon = (Real)0.0;

    // Clear out the index array in case this function is called more than
    // one time (for different dimThreshold or uncertainty parameters).
    delete[] m_aiHIndex;
    m_aiHIndex = 0;

    // Clear out the line parameters in case the hull turns out not to be
    // linear.
    m_kLineOrigin = Vector2<Real>::ZERO;
    m_kLineDirection = Vector2<Real>::ZERO;

    // Determine the dimension of the point set.
    Mapper2<Real> kMapper(m_iVQuantity,m_akVertex,fEpsilon);

    if ( kMapper.GetDimension() == 0 )
    {
        // The hull is (nearly) a point.
        m_iHDim = 0;
        m_iHQuantity = 1;
        m_aiHIndex = new int[1];
        m_aiHIndex[0] = 0;
        return true;
    }

    int i0 = kMapper.GetExtremeIndex(0);
    int i1 = kMapper.GetExtremeIndex(1);
    if ( kMapper.GetDimension() == 1 )
    {
        // The hull is (nearly) a line segment.
        m_iHDim = 1;
        m_iHQuantity = 2;
        m_aiHIndex = new int[2];
        m_aiHIndex[0] = i0;
        m_aiHIndex[1] = i1;
        m_kLineOrigin = kMapper.GetOrigin();
        m_kLineDirection = kMapper.GetDirection(0);
        return true;
    }

    int i2 = kMapper.GetExtremeIndex(2);
    m_iHDim = 2;

    int i;

#ifndef USE_QUERY2TRATIONAL
    // Transform the vertices to the square [-2^{20},2^{20}]^2 to allow use
    // of 64-bit integers for hull construction.
    if ( !m_akSVertex )
        m_akSVertex = new Vector2<Real>[m_iVQuantity];

    Real fScale = ((Real)(1 << 20))/kMapper.GetMaxRange();
    for (i = 0; i < m_iVQuantity; i++)
        m_akSVertex[i] = (m_akVertex[i] - kMapper.GetMin())*fScale;

#ifdef USE_QUERY2INT64
    m_pkQuery = new Query2Int64<Real>(m_iVQuantity,m_akSVertex);
#else
    m_pkQuery = new Query2TInteger<Real>(m_iVQuantity,m_akSVertex);
#endif
#else
    m_akSVertex = (Vector2<Real>*)m_akVertex;
    m_pkQuery = new Query2TRational<Real>(m_iVQuantity,m_akSVertex);
#endif

    HullEdge2<Real>* pkE0;
    HullEdge2<Real>* pkE1;
    HullEdge2<Real>* pkE2;

    if ( kMapper.GetExtremeCCW() )
    {
        pkE0 = new HullEdge2<Real>(i0,i1);
        pkE1 = new HullEdge2<Real>(i1,i2);
        pkE2 = new HullEdge2<Real>(i2,i0);
    }
    else
    {
        pkE0 = new HullEdge2<Real>(i0,i2);
        pkE1 = new HullEdge2<Real>(i2,i1);
        pkE2 = new HullEdge2<Real>(i1,i0);
    }

    pkE0->Insert(pkE2,pkE1);
    pkE1->Insert(pkE0,pkE2);
    pkE2->Insert(pkE1,pkE0);

    HullEdge2<Real>* pkHull = pkE0;
    for (i = 0; i < m_iVQuantity; i++)
    {
        if ( !Update(pkHull,i) )
        {
            pkHull->DeleteAll();
            return false;
        }
    }

    pkHull->GetIndices(m_iHQuantity,m_aiHIndex);
    pkHull->DeleteAll();

    delete m_pkQuery;
    m_pkQuery = 0;
    return true;
}
//----------------------------------------------------------------------------
template <class Real>
int ConvexHull2<Real>::GetDimension () const
{
    return m_iHDim;
}
//----------------------------------------------------------------------------
template <class Real>
int ConvexHull2<Real>::GetQuantity () const
{
    return m_iHQuantity;
}
//----------------------------------------------------------------------------
template <class Real>
int* ConvexHull2<Real>::GetIndices (bool bTakeOwnership)
{
    int* aiIndices = m_aiHIndex;
    if ( bTakeOwnership )
        m_aiHIndex = 0;

    return aiIndices;
}
//----------------------------------------------------------------------------
template <class Real>
const Vector2<Real>& ConvexHull2<Real>::GetLineOrigin () const
{
    return m_kLineOrigin;
}
//----------------------------------------------------------------------------
template <class Real>
const Vector2<Real>& ConvexHull2<Real>::GetLineDirection () const
{
    return m_kLineDirection;
}
//----------------------------------------------------------------------------
template <class Real>
bool ConvexHull2<Real>::Update (HullEdge2<Real>*& rpkHull, int i)
{
    // Locate an edge visible to the input point (if possible).
    HullEdge2<Real>* pkVisible = 0;
    HullEdge2<Real>* pkCurrent = rpkHull;
    do
    {
        if ( pkCurrent->GetSign(i,m_pkQuery) > 0 )
        {
            pkVisible = pkCurrent;
            break;
        }

        pkCurrent = pkCurrent->A[1];
    }
    while (pkCurrent != rpkHull);

    if ( !pkVisible )
    {
        // The point is inside the current hull; nothing to do.
        return true;
    }

    // Remove the visible edges.
    HullEdge2<Real>* pkAdj0 = pkVisible->A[0];
    assert( pkAdj0 );
    if ( !pkAdj0 )
        return false;

    HullEdge2<Real>* pkAdj1 = pkVisible->A[1];
    assert( pkAdj1 );
    if ( !pkAdj1 )
        return false;

    pkVisible->DeleteSelf();

    while ( pkAdj0->GetSign(i,m_pkQuery) > 0 )
    {
        rpkHull = pkAdj0;
        pkAdj0 = pkAdj0->A[0];
        assert( pkAdj0 );
        if ( !pkAdj0 )
            return false;

        pkAdj0->A[1]->DeleteSelf();
    }

    while ( pkAdj1->GetSign(i,m_pkQuery) > 0 )
    {
        rpkHull = pkAdj1;
        pkAdj1 = pkAdj1->A[1];
        assert( pkAdj1 );
        if ( !pkAdj1 )
            return false;

        pkAdj1->A[0]->DeleteSelf();
    }

    // Insert the new edges formed by the input point and the end points of
    // the polyline of invisible edges.
    HullEdge2<Real>* pkEdge0 = new HullEdge2<Real>(pkAdj0->V[1],i);
    HullEdge2<Real>* pkEdge1 = new HullEdge2<Real>(i,pkAdj1->V[0]);
    pkEdge0->Insert(pkAdj0,pkEdge1);
    pkEdge1->Insert(pkEdge0,pkAdj1);
    rpkHull = pkEdge0;

    return true;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// explicit instantiation
//----------------------------------------------------------------------------
// float
template WM3_ITEM
class ConvexHull2<float>;

// double
template WM3_ITEM
class ConvexHull2<double>;
//----------------------------------------------------------------------------
}


