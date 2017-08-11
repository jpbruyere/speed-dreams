#ifndef _PARAMETRICCUBIC_H_
#define _PARAMETRICCUBIC_H_

#include "Cubic.h"
#include <tgf.h>

class ParametricCubic  
{
public:
	ParametricCubic();
	~ParametricCubic();

	void	Set( glm::dvec2 p0, glm::dvec2 p1, glm::dvec2 v0, glm::dvec2 v1 );

	glm::dvec2	Calc( double t ) const;
	glm::dvec2	CalcGradient( double t ) const;
	double	CalcCurvature( double t ) const;

private:
	Cubic	m_x;
	Cubic	m_y;
};

#endif
