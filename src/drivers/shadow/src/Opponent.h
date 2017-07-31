/***************************************************************************

    file        : Opponent.h
    created     : 18 Apr 2006
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

#ifndef _OPPONENT_H_
#define _OPPONENT_H_

#include <car.h>
#include <raceman.h>

#include "PathRecord.h"
#include "TeamInfo.h"
#include "Span.h"

class TDriver;

class Opponent  
{
public:
	enum	// flags
	{
		F_LEFT			= 0x000001,
		F_RIGHT			= 0x000002,
		F_FRONT			= 0x000004,
		F_REAR			= 0x000008,

		F_AHEAD			= 0x000010,
		F_TO_SIDE		= 0x000020,
		F_BEHIND		= 0x000040,

		F_TRK_LEFT		= 0x000100,
		F_TRK_RIGHT		= 0x000200,

		F_CATCHING		= 0x001000,
		F_CATCHING_ACC	= 0x002000,
		F_COLLIDE		= 0x004000,
		F_TRAFFIC		= 0x008000,
		F_CLOSE			= 0x010000,
		F_TEAMMATE		= 0x020000,
		F_LAPPER		= 0x040000,	// it's lapping us.
		F_BEING_LAPPED	= 0x080000,	// we're lapping it.
		F_DANGEROUS		= 0x100000,
	};

	struct Sit
	{
		// t = local track relative.
		// r = relative to velocity frame of my car.
		// g = global torcs coordinate frame.
		// d = delta or difference.
		// a = average.

		double	spd;

		double	tVX;	// along track.
		double	tVY;	// normal to track.
		double	tYaw;

		double	rdPX;
		double	rdPY;
		double	rdVX;
		double	rdVY;

		double	agVX;
		double	agVY;
		double	ragVX;
		double	ragVY;

		double	arAX;
		double	arAY;
		double	agAX;
		double	agAY;
		double	ragAX;
		double	ragAY;

		double	minDX;
		double	minDY;

		double	relPos;
		double	offs;
	};

	struct PassInfo
	{
		bool	isSpace;
		double	offset;
		double	mySpeed;
		bool	goodPath;
		double	bestU;
		double	bestV;
		double	myOffset;
	};

	struct Info
	{
		Info() { memset( this, 0, sizeof(*this) ); }

		bool		GotFlags( int f ) const	{ return (flags & f) == f; }

		int			flags;		// flags from above (cOPP_XXX).

		Sit			sit;

		double		avoidLatchTime;
		double		dangerousLatchTime;

		double		catchTime;
		double		catchY;
		double		catchSpd;
		double		catchDecel;

		double		catchAccTime;
		double		catchAccY;
		double		catchAccSpd;

		bool		newCatching;
		double		newCatchSpd;
		double		newCatchTime;
		double		newAheadTime;
		double		newMidPos;
		double		newBestOffset;
		PassInfo	newPiL;
		PassInfo	newPiR;

		double	tmDamage;
	};

public:
	Opponent();
	~Opponent();

	void		Initialise( MyTrack* pTrack, CarElt* pCar );

	CarElt*		GetCar();
	const Info&	GetInfo() const;
	Info&		GetInfo();

	void		UpdatePath();

    void		UpdateSit( const CarElt* myCar, const TeamInfo* pTeamInfo, double myDirX, double myDirY );
    void		ProcessMyCar( const Situation* s, const TeamInfo* pTeamInfo, const CarElt* myCar, const Sit& mySit,
                              const TDriver& me, double myMaxAccX, int idx );

private:
	Info			m_info;
	PathRecord		m_path;		// info about path of this opponent.
};

#endif
