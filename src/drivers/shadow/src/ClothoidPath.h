/***************************************************************************

    file                 : ClothoidPath.h
    created              : 9 Apr 2006
    copyright            : (C)2006 Tim Foden - (C)2014 Xavier BERTAUX
    email                : bertauxx@yahoo.fr
    version              : $Id: ClothoidPath.h 5631 2014-12-27 21:32:55Z torcs-ng $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef _CLOTHOIDPATH_H_
#define _CLOTHOIDPATH_H_

#include "LinePath.h"
#include "MyTrack.h"
#include "Array.h"

class ClothoidPath : public LinePath
{
public:
	enum
	{
		FLAG_FLYING		= 0x01,
	};

	struct Options
	{
		int		bumpMod;
		double	maxL;
		double	maxR;

		Options() : bumpMod(0), maxL(999), maxR(999) {}
        Options( int bm, double ml = 999, double mr = 999 )	:	bumpMod(bm), maxL(ml), maxR(mr) {}
	};

public:
	ClothoidPath();
	virtual ~ClothoidPath();

	void	ClearFactors();
	void	AddFactor( double factor );
	void	SetFactors( const Array<double>& factors );
	const Array<double>&	GetFactors() const;

    void	MakeSmoothPath( MyTrack* pTrack, const CarModel& cm, const Options& opts );

private:
	void	AnalyseBumps( const CarModel& cm, bool dumpInfo = false );
	void	SmoothBetween( int step );
	void	SetOffset( const CarModel& cm, double k, double t,
					   PathPt* l3, const PathPt* l2, const PathPt* l4 );

	void	OptimiseLine( const CarModel& cm, int idx, int step, double hLimit,
						  PathPt* l3, const PathPt* l2, const PathPt* l4 );

	void	Optimise(	const CarModel& cm, double factor,
						int idx, PathPt* l3,
						const PathPt* l0, const PathPt* l1,
						const PathPt* l2, const PathPt* l4,
						const PathPt* l5, const PathPt* l6,
						int	bumpMod );

    void	OptimisePath(	const CarModel& cm,	int step, int nIterations, int bumpMod );

private:
	Array<double>	m_factors;
};

#endif
