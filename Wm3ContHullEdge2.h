// Geometric Tools
// http://www.geometrictools.com
// Copyright (c) 1998-2005.  All Rights Reserved
//
// The Wild Magic Library (WM3) source code is supplied under the terms of
// the license agreement
//     http://www.geometrictools.com/License/WildMagic3License.pdf
// and may not be copied or disclosed except in accordance with the terms
// of that agreement.

#ifndef WM3CONTHULLEDGE2_H
#define WM3CONTHULLEDGE2_H

#include "Wm3Query2.h"

namespace Wm3
{

template <class Real>
class WM3_ITEM HullEdge2
{
public:
    HullEdge2 (int iV0, int iV1);

    int GetSign (int i, const Query2<Real>* pkQuery);

    void Insert (HullEdge2* pkAdj0, HullEdge2* pkAdj1);
    void DeleteSelf ();
    void DeleteAll ();

    void GetIndices (int& riHQuantity, int*& raiHIndex);

    int V[2];
    HullEdge2* A[2];
    int Sign;
    int Time;
};

}

#endif


