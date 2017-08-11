/***************************************************************************

    file        : LinearRegression.h
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

#ifndef _LINEARREGRESSION_H_
#define _LINEARREGRESSION_H_

#include <tgf.h>

class LinearRegression  
{
public:
	LinearRegression();
	~LinearRegression();

	void	Clear();
	void	Sample( double X, double Y );
	void	Sample( const glm::dvec2& p );
	double	CalcY( double X ) const;
	void	CalcLine( glm::dvec2& p, glm::dvec2& v ) const;

public:
	int		m_n;
	double	m_sumX;
	double	m_sumY;
	double	m_sumXY;
	double	m_sumXX;
	double	m_sumYY;
};

#endif
