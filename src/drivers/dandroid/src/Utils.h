/***************************************************************************

    file        : Utils.h
    created     : 9 Apr 2006
    copyright   : (C) 2006 Tim Foden

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/


#ifndef _UTILS_H_
#define _UTILS_H_

#include "Vec2d.h"
#include "Vec3d.h"

#define MN(x, y)	((x) < (y) ? (x) : (y))
#define MX(x, y)	((x) > (y) ? (x) : (y))

class Utils  
{
  public:
  Utils();
  ~Utils();

  static double	ClosestPtOnLine( double ptx, double pty, double px, double py, double vx, double vy );
  static double	DistPtFromLine( double ptx, double pty, double px, double py, double vx, double vy );

  static bool	LineCrossesLine( double p0x, double p0y, double v0x, double v0y, double p1x, double p1y, double v1x, double v1y, double& t );
  static bool	LineCrossesLine( const glm::dvec2& p0, const glm::dvec2& v0, const glm::dvec2& p1, const glm::dvec2& v1, double& t );
  static bool	LineCrossesLineXY( const glm::dvec3& p0, const glm::dvec3& v0, const glm::dvec3& p1, const glm::dvec3& v1, double& t );

  static bool	LineCrossesLine( const glm::dvec2& p0, const glm::dvec2& v0, const glm::dvec2& p1, const glm::dvec2& v1, double& t0, double& t1 );

  static double	CalcCurvature( double p1x, double p1y, double p2x, double p2y, double p3x, double p3y );
  static double	CalcCurvature( const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3 );
  static double	CalcCurvatureTan( const glm::dvec2& p1, const glm::dvec2& tangent, const glm::dvec2& p2 );
  static double	CalcCurvatureXY( const glm::dvec3& p1, const glm::dvec3& p2, const glm::dvec3& p3 );
  static double	CalcCurvatureZ( const glm::dvec3& p1, const glm::dvec3& p2, const glm::dvec3& p3 );

  static bool		CalcTangent( const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3, glm::dvec2& tangent );

  static double	InterpCurvatureRad( double k0, double k1, double t );
  static double	InterpCurvatureLin( double k0, double k1, double t );
  static double	InterpCurvature( double k0, double k1, double t );

  static double	VecAngXY( const glm::dvec3& v );
  static double	VecLenXY( const glm::dvec3& v );
  static glm::dvec3	VecNormXY( const glm::dvec3& v );

  static double	VecAngle( const glm::dvec2& v );
  static glm::dvec2	VecNorm( const glm::dvec2& v );
  static glm::dvec2	VecUnit( const glm::dvec2& v );
};

#endif // _UTILS_H_
