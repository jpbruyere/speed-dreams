// ParametricCubic.cpp: implementation of the ParametricCubic class.
//
//////////////////////////////////////////////////////////////////////

#include "ParametricCubic.h"
#include "Utils.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

ParametricCubic::ParametricCubic()
{
}

ParametricCubic::~ParametricCubic()
{
}

void ParametricCubic::Set( Vec2d p0, Vec2d p1, Vec2d p2, Vec2d p3 )
//void	ParametricCubic::Set( Vec2d p0, Vec2d v0, Vec2d p1, Vec2d v1 )
{
//	double	v0Len = 1;//v0.len();
//	double	v1Len = 1;//v1.len();
//	m_x.Set( 0, p0.x, v0.x / v0Len, 1, p1.x, v1.x / v1Len );
//	m_y.Set( 0, p0.y, v0.y / v0Len, 1, p1.y, v1.y / v1Len );

	Vec2d	v1, v2;
	Utils::CalcTangent( p0, p1, p2, v1 );
	Utils::CalcTangent( p1, p2, p3, v2 );
	double	len = (p2 - p1).len();// * 1.15;
	v1 = v1 * len;
	v2 = v2 * len;
	m_x.Set( 0, p1.x, v1.x, 1, p2.x, v2.x );
	m_y.Set( 0, p1.y, v1.y, 1, p2.y, v2.y );
}

Vec2d ParametricCubic::Calc( double t ) const
{
	double	x = m_x.CalcY(t);
	double	y = m_y.CalcY(t);
	return Vec2d(x, y);
}

Vec2d ParametricCubic::CalcGradient( double t ) const
{
	double	dx = m_x.CalcGradient(t);
	double	dy = m_y.CalcGradient(t);
	return Vec2d(dx, dy);
}

double ParametricCubic::CalcCurvature( double t ) const
{
	// signed curvature....
	//
	//          x'y" - y'x"
	//	K = -------------------
	//      (x'^2 + y'^2)^(3/2)

	double	x1d = m_x.CalcGradient(t);
	double	x2d = m_x.Calc2ndDerivative(t);
	double	y1d = m_y.CalcGradient(t);
	double	y2d = m_y.Calc2ndDerivative(t);

	double	k = (x1d * y2d - y1d * x2d) / pow(x1d * x1d + y1d * y1d, 3.0 / 2);

//	Vec2d	p0 = Calc(0);
//	Vec2d	pp = Calc(t);
//	Vec2d	p1 = Calc(1);
//	double	k2 = Utils::CalcCurvature(p0, pp, p1);

	return k;
}
