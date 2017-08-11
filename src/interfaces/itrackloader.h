/***************************************************************************
                 itrackloader.h -- Interface for track loaders

    created              : Wed Mar 31 22:12:01 CEST 2011
    copyright            : (C) 2011 by Jean-Philippe Meuret                         
    web                  : http://www.speed-dreams.org
    version              : $Id$
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

/** @file   
    	Interface for track loaders
    @version	$Id$
*/

#ifndef __ITRACKLOADER__H__
#define __ITRACKLOADER__H__

#include <track.h>


class ITrackLoader
{
 public:

	virtual tTrack* load(const char* filename, bool grExts = false) = 0;
	virtual void unload() = 0;

	// ???? a ITrack interface ??????
	//virtual float globalHeight(tTrackSeg*, float x, float y) = 0;
	//virtual float localHeight(tTrkLocPos* pos) = 0;
 	//virtual void global2Local(tTrackSeg* seg, float x, float y, tTrkLocPos* pos, int sides) = 0;
 	//virtual void local2Global(tTrkLocPos* pos, float* x, float* y) = 0;
 	//virtual void sideNormal(tTrackSeg* seg, float x, float y, int side, glm::vec3* norm) = 0;
	//virtual void surfaceNormal(tTrkLocPos* pos, glm::vec3* norm) = 0;
};

#endif // __ITRACKLOADER__H__
