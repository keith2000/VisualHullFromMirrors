// Geometric Tools
// http://www.geometrictools.com
// Copyright (c) 1998-2005.  All Rights Reserved
//
// The Wild Magic Library (WM3) source code is supplied under the terms of
// the license agreement
//     http://www.geometrictools.com/License/WildMagic3License.pdf
// and may not be copied or disclosed except in accordance with the terms
// of that agreement.

#ifndef WM3PLATFORMS_H
#define WM3PLATFORMS_H

// Platform-specific information goes in this header file.  The defines to
// control which platform is included are:
//
// MINGW      :  Minimalist GNU for Windows
// WIN32      :  Microsoft Windows 2000/XP
// __APPLE__  :  Macintosh OS X (10.2.3 or higher required)
// <none>     : Linux
//
// Add others as needed.

//----------------------------------------------------------------------------
// Minimalist GNU for Windows
//----------------------------------------------------------------------------
#if defined(MINGW)

// Macro used for Microsoft Windows systems to support dynamic link libraries.
// Not needed for MINGW.
#define WM3_ITEM

typedef long long Integer64;

//----------------------------------------------------------------------------
// Microsoft Windows 2000/XP platform
//----------------------------------------------------------------------------
#elif defined(WIN32)

// for a DLL library
#if defined(WM3_DLL_EXPORT)
#define WM3_ITEM __declspec(dllexport)

// for a client of the DLL library
#elif defined(WM3_DLL_IMPORT)
#define WM3_ITEM __declspec(dllimport)

// for a static library
#else
#define WM3_ITEM

#endif

#if defined(_MSC_VER)

// Microsoft Visual C++ specific pragmas.  MSVC6 is version 12.00, MSVC7.0 is
// version 13.00, and MSVC7.1 is version 13.10.
#if _MSC_VER < 1300
#define WM3_USING_VC6
#elif _MSC_VER < 1310
#define WM3_USING_VC70
#else
#define WM3_USING_VC71
#endif

#if defined(WM3_USING_VC6)

// Disable the warning "non dll-interface class FOO used as base for
// dll-interface class BAR."  These occur in the derivations
// class Binary2D : public ImageInt2D; class Binary3D : public ImageInt3D;
#pragma warning( disable : 4275 )

// Disable the warning about truncating the debug names to 255 characters.
// This warning shows up often with STL code in MSVC6, but not MSVC7.
#pragma warning( disable : 4786 )

// This warning is disabled because MSVC6 warns about not finding
// implementations for the pure virtual functions that occur in the template
// classes 'template <class Real>' when explicity instantiating the classe.
// NOTE:  If you create your own template classes that will be explicitly
// instantiated, you should re-enable the warning to make sure that in fact
// all your member data and functions have been defined and implemented.
#pragma warning( disable : 4661 )

#endif

// The use of WM3_ITEM to export an entire class generates warnings when
// member data and functions involving templates or inlines occur.  To avoid
// the warning, WM3_ITEM can be applied only to those items that really need
// to be exported.  I do not want to go to that effort...
#pragma warning( disable : 4251 ) 

// Enable the warning: "loop control variable declared in the for-loop is
// used outside the for-loop scope.  The default level 3 warnings do not
// enable this (level 4 does), but should since allowing the outside scope
// is a Microsoft extension.
// #pragma warning( error : 4289 )

typedef __int64 Integer64;

#endif

// Specialized instantiation of static members in template classes before or
// after the class itself is instantiated is not a problem with Visual Studio
// .NET 2003 (VC 7.1), but VC 6 likes the specialized instantiation to occur
// after the class instantiation.
// #define WM3_INSTANTIATE_BEFORE
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// Macintosh OS X platform
//----------------------------------------------------------------------------
#elif defined(__APPLE__)

#define WM3_BIG_ENDIAN

// Macro used for Microsoft Windows systems to support dynamic link libraries.
// Not needed for the Macintosh.
#define WM3_ITEM

#include <stdint.h>
typedef int64_t Integer64;

// g++ wants specialized template instantiations to occur after the explicit
// class instantiations.  CodeWarrior wants them to occur before.
#ifdef __MWERKS__
#define WM3_INSTANTIATE_BEFORE
#endif
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// Linux platform
//----------------------------------------------------------------------------
#else

// Macro used for Microsoft Windows systems to support dynamic link libraries.
// Not needed for Linux.
#define WM3_ITEM

#include <stdint.h>
typedef int64_t Integer64;

// Linux on a PC. Red Hat 8.x g++ has problems with specialized instantiation
// of static members in template classes *before* the class itself is
// explicitly instantiated.  The problem is not consistent; for example, Math
// Vector*, and Matrix* classes compile fine, but not Integrate1 or
// BSplineRectangle.  So the following macro is *not* defined for this
// platform.  If you have a Linux system that does appear to require the
// instantiation before, then enable this macro.
// #define WML_INSTANTIATE_BEFORE

#endif
//----------------------------------------------------------------------------

#endif


