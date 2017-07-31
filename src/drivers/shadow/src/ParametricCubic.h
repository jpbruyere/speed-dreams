#ifndef _PARAMETRICCUBIC_H_
#define _PARAMETRICCUBIC_H_

#include "Cubic.h"
#include "Vec2d.h"

class ParametricCubic  
{
public:
	ParametricCubic();
	~ParametricCubic();

	void	Set( Vec2d p0, Vec2d p1, Vec2d v0, Vec2d v1 );

	Vec2d	Calc( double t ) const;
	Vec2d	CalcGradient( double t ) const;
	double	CalcCurvature( double t ) const;

private:
	Cubic	m_x;
	Cubic	m_y;
};

#endif
