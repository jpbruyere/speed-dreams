/***************************************************************************

    file        : Cubic.h
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

#ifndef _CUBIC_H_
#define _CUBIC_H_

class Cubic  
{
public:
	Cubic();
	Cubic( double a, double b, double c, double d );
	Cubic(	double x0, double y0, double s0,
			double x1, double y1, double s1 );
	~Cubic();

	void	Set( double a, double b, double c, double d );
	void	Set( double x0, double y0, double s0,
				 double x1, double y1, double s1 );

	double	CalcY( double x ) const;
	double	CalcGradient( double x ) const;
	double	Calc2ndDerivative( double x ) const;

public:
	double	m_coeffs[4];	// coefficients
};

#endif
