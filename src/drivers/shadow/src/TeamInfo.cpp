/***************************************************************************

    file        : TeamInfo.cpp
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

// TeamInfo.cpp: implementation of the TeamInfo class.
//
//////////////////////////////////////////////////////////////////////

#include "TeamInfo.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

TeamInfo::TeamInfo()
:	m_size(0),
	m_ppItems(0)
{
}

TeamInfo::~TeamInfo()
{
	Empty();
}

void TeamInfo::Empty()
{
	if( m_ppItems )
	{
		for( int i = 0; i < m_size; i++ )
			delete m_ppItems[i];
		delete [] m_ppItems;
		m_size = 0;
		m_ppItems = 0;
	}
}

void TeamInfo::Add( int index, Item* pItem )
{
	if( index >= m_size )
	{
		// expand array.
		Item**	ppNewItems = new Item*[index + 1];
		int	i;
		for( i = 0; i < m_size; i++ )
			ppNewItems[i] = m_ppItems[i];
		for( ; i <= index; i++ )
			ppNewItems[i] = 0;

		delete [] m_ppItems;
		m_ppItems = ppNewItems;
		m_size = index + 1;
	}

	if( m_ppItems[index] )
		delete m_ppItems[index];
	m_ppItems[index] = pItem;

	// see if we can find a team mate.
//	if( pItem->team >= 0 )
	{
		for( int i = 0; i < m_size; i++ )
		{
			if( i != index && m_ppItems[i] &&
				strcmp(m_ppItems[i]->teamName, pItem->teamName) == 0 &&
				m_ppItems[i]->pOther == 0 )
			{
				// found a team-mate.
				pItem->pOther = m_ppItems[i];
				m_ppItems[i]->pOther = pItem;
				break;
			}
		}
	}
}

const TeamInfo::Item* TeamInfo::GetAt( int index ) const
{
	return m_ppItems[index];
}

TeamInfo::Item*	TeamInfo::GetAt( int index )
{
	return m_ppItems[index];
}

bool TeamInfo::IsTeamMate( const CarElt* pCar0, const CarElt* pCar1 ) const
{
//	const Item*	pItem0 = GetAt(pCar0->index);
//	const Item*	pItem1 = pItem0 ? pItem0->pOther : 0;
//	return pItem1 ? (pItem1->pCar == pCar1) : false;
//	return false;
	return strcmp(pCar0->_teamname, pCar1->_teamname) == 0;
}
