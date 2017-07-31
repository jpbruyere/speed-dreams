/***************************************************************************

    file        : PathRecord.h
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

#ifndef _PATHRECORD_H_
#define _PATHRECORD_H_

#include "MyTrack.h"

class PathRecord
{
public:
	class Rec
	{
	public:
		const Seg*	pSeg;	// MyTrack segment this info applies to.
		double		avgW;	// average position
		double		avgV;	// average speed
	};

public:
	PathRecord();
	~PathRecord();

	void		Initialise( MyTrack* pTrack, CarElt* pCar );
	void		Update();

	tCarElt*	GetCar();
	MyTrack*	GetTrack();
	void		GetPredictionForPos( double pos, double& w, double& v ) const;
	void		GetPrediction( double& w, double& v ) const;
	double		CalcConfidence( double w, double v ) const;
	double		CalcConfidence() const;

	const Rec&	GetAt( int index ) const;

public:
	MyTrack*	m_pTrack;
	tCarElt*	m_pCar;

	Rec*		m_pData;
	int			m_lastSeg;
	Vec2d		m_lastPt;
	double		m_lastSpd;
};

#endif
