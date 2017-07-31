/***************************************************************************

    file        : Shared.h
    created     : 19 Apr 2006
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

#ifndef _SHARED_H_
#define _SHARED_H_

#include <track.h>

#include "TeamInfo.h"
#include "ClothoidPath.h"

class Shared  
{
public:
	Shared();
	~Shared();

public:
	TeamInfo		m_teamInfo;
	tTrack*			m_pTrack;
	ClothoidPath	m_path[3];
};

#endif
