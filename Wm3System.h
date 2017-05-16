// Geometric Tools
// http://www.geometrictools.com
// Copyright (c) 1998-2005.  All Rights Reserved
//
// The Wild Magic Library (WM3) source code is supplied under the terms of
// the license agreement
//     http://www.geometrictools.com/License/WildMagic3License.pdf
// and may not be copied or disclosed except in accordance with the terms
// of that agreement.

#ifndef WM3SYSTEM_H
#define WM3SYSTEM_H

#include "Wm3Platforms.h"

// common standard library headers
#include <cassert>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <iostream>
#include <fstream>

// template classes
#include "Wm3TArray.h"
#include "Wm3THashSet.h"
#include "Wm3THashTable.h"
#include "Wm3TList.h"
#include "Wm3TSet.h"
#include "Wm3TStack.h"


namespace Wm3
{

class WM3_ITEM System
{
public:
    // swap byte order (size = 2, 4, or 8)
    static void SwapBytes (int iSize, void* pvValue);
    static void SwapBytes (int iSize, int iQuantity, void* pvValue);

    // The output is stored in little endian format.
    static bool IsBigEndian ();
    static void EndianCopy (int iSize, const void* pvSrc, void* pvDst);
    static void EndianCopy (int iSize, int iQuantity, const void* pvSrc,
        void* pvDst);

    // time utility (return value is seconds)
    static double GetTime ();

    // File support for buffer load and save.
    static bool Load (const char* acFilename, char*& racBuffer, int& riSize);
    static bool Save (const char* acFilename, const char* acBuffer,
        int iSize);
    static bool Append (const char* acFilename, char* acBuffer, int iSize);

    // Buffer read and write, file read and write, for character data.  The
    // endianness is irrelevant.
    static int Read1 (const char* acBuffer, int iQuantity, void* pvData);
    static int Write1 (char* acBuffer, int iQuantity, const void* pvData);
    static int Read1 (FILE* pkFile, int iQuantity, void* pvData);
    static int Write1 (FILE* pkFile, int iQuantity, const void* pvData);

    // Buffer read and write, file read and write.  The operations store the
    // results in little-endian order.  The return value is the number of
    // bytes read or written.  The function name suffix indicates the number
    // of bytes transferred per item.
    static int Read2le (const char* acBuffer, int iQuantity, void* pvData);
    static int Read4le (const char* acBuffer, int iQuantity, void* pvData);
    static int Read8le (const char* acBuffer, int iQuantity, void* pvData);
    static int Write2le (char* acBuffer, int iQuantity, const void* pvData);
    static int Write4le (char* acBuffer, int iQuantity, const void* pvData);
    static int Write8le (char* acBuffer, int iQuantity, const void* pvData);
    static int Read2le (FILE* pkFile, int iQuantity, void* pvData);
    static int Read4le (FILE* pkFile, int iQuantity, void* pvData);
    static int Read8le (FILE* pkFile, int iQuantity, void* pvData);
    static int Write2le (FILE* pkFile, int iQuantity, const void* pvData);
    static int Write4le (FILE* pkFile, int iQuantity, const void* pvData);
    static int Write8le (FILE* pkFile, int iQuantity, const void* pvData);

    // Buffer read and write, file read and write.  The operations store the
    // results in big-endian order.  The return value is the number of
    // bytes read or written.  The function name suffix indicates the number
    // of bytes transferred per item.
    static int Read2be (const char* acBuffer, int iQuantity, void* pvData);
    static int Read4be (const char* acBuffer, int iQuantity, void* pvData);
    static int Read8be (const char* acBuffer, int iQuantity, void* pvData);
    static int Write2be (char* acBuffer, int iQuantity, const void* pvData);
    static int Write4be (char* acBuffer, int iQuantity, const void* pvData);
    static int Write8be (char* acBuffer, int iQuantity, const void* pvData);
    static int Read2be (FILE* pkFile, int iQuantity, void* pvData);
    static int Read4be (FILE* pkFile, int iQuantity, void* pvData);
    static int Read8be (FILE* pkFile, int iQuantity, void* pvData);
    static int Write2be (FILE* pkFile, int iQuantity, const void* pvData);
    static int Write4be (FILE* pkFile, int iQuantity, const void* pvData);
    static int Write8be (FILE* pkFile, int iQuantity, const void* pvData);

    // This is needed on the Macintosh because of its complicated application
    // structure.  In particular, this function is used in Xcode projects and
    // ignores the directory entry, but assumes that the data files required
    // by an application are added to the Resources folder of the projects.
    // The other platforms concatenate the directory and filename, the result
    // stored in class-static memory (so be careful with threads).
    static const char* GetPath (const char* acDirectory,
        const char* acFilename);

    // Creation of colors, hides endianness.
    static unsigned int MakeRGB (unsigned char ucR, unsigned char ucG,
        unsigned char ucB);
    static unsigned int MakeRGBA (unsigned char ucR, unsigned char ucG,
        unsigned char ucB, unsigned char ucA);

private:
    enum { SYSTEM_MAX_PATH = 1024 };
    static char ms_acPath[SYSTEM_MAX_PATH];
};


// Allocation and deallocation of 2D arrays.  On deallocation, the array
// pointer is set to null.
template <class T> void Allocate (int iCols, int iRows, T**& raatArray);
template <class T> static void Deallocate (T**& raatArray);

// Allocation and deallocation of 3D arrays.  On deallocation, the array
// pointer is set to null.
template <class T> void Allocate (int iCols, int iRows, int iSlices,
    T***& raaatArray);
template <class T> void Deallocate (T***& raaatArray);

#include "Wm3System.inl"

}

#endif


