#ifndef _AVOIDANCE_H_
#define _AVOIDANCE_H_

#include "Opponent.h"
#include "Span.h"
#include <car.h>

class TDriver;

class Avoidance  
{
public:
	enum
	{
		PRI_IGNORE	= 0,
		PRI_GENERIC	= 1,
		PRI_TRAFFIC = 3,
		PRI_LAPPING	= 5,
		PRI_MAX		= 10,
	};

	struct Info
	{
		int				flags;
		int				avoidAhead;
//		int				nAvoidAhead;
		int				avoidToSide;
		int				avoidLapping;
		double			k;
		double			nextK;
		double			spdL;
		double			spdR;
		double			spdF;
		double			minLSideDist;
		double			minRSideDist;
		double			minLDist;
		double			minRDist;
		double			bestPathOffs;
		Span			aheadSpan;
		Span			sideSpan;
		Opponent::Info*	pClosestAhead;

		Info()
		{
			Init();
		}

		void	Init()
		{
			flags = 0;
			avoidAhead = 0;
//			nAvoidAhead = 0;
			avoidToSide = 0;
			avoidLapping = 0;
			k = 0;
			nextK = 0;
			spdL = 200;
			spdR = 200;
			spdF = 200;
			minLSideDist = INT_MAX;
			minRSideDist = INT_MAX;
			minLDist = INT_MAX;
			minRDist = INT_MAX;
			bestPathOffs = 0;
			aheadSpan = Span(-999, 999);
			sideSpan = Span(-999, 999);
			pClosestAhead = 0;
		}
	};

public:
	Avoidance();
	virtual ~Avoidance();

	virtual int		priority( const Info& ai, const CarElt* pCar ) const = 0;
	virtual Vec2d	calcTarget( const Info& ai, const CarElt* pCar,
                                const TDriver& me ) = 0;
};

#endif
