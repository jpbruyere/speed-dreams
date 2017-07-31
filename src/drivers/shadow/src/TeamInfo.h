/***************************************************************************

    file        : TeamInfo.h
    created     : 21 Apr 2006
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

#ifndef _TEAMINFO_H_
#define _TEAMINFO_H_

#include <car.h>

class TeamInfo  
{
public:
	enum
	{
		PIT_NOT_SHARED,
		PIT_NOT_REQUIRED,
		PIT_REQUEST,
		PIT_REQUEST_URGENT,	// else will likely run out of fuel.
		PIT_IN_USE,
	};

	struct	Item
	{
		int			index;		// index of car in race.
		const char*	teamName;	// name of team.
		int			damage;		// damage of this team member.
		int			pitState;	// request for shared pit.
		Item*		pOther;		// the other team member.
		CarElt*		pCar;		// the car of this team member.
	};

public:
	TeamInfo();
	~TeamInfo();

	void		Empty();
	void		Add( int index, Item* pItem );
	const Item*	GetAt( int index ) const;
	Item*		GetAt( int index );

	bool		IsTeamMate( const CarElt* pCar0, const CarElt* pCar1 ) const;

private:
	int			m_size;
	Item**		m_ppItems;
};

#endif // !defined(AFX_TEAMINFO_H__7EA9649D_1527_4B70_BA9A_63E73AEFC9FF__INCLUDED_)
