/***************************************************************************

    file        : Path.h
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

#ifndef _PATH_H_
#define _PATH_H_

#include "PtInfo.h"

class Path
{
public:
	Path();
	virtual ~Path();

	virtual bool	ContainsPos( double trackPos ) const = 0;
	virtual bool	GetPtInfo( double trackPos, PtInfo& pi ) const = 0;
};

#endif
