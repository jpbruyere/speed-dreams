/***************************************************************************

    file                 : trackutil.cpp
    created              : Sun Jan 30 22:58:00 CET 2000
    copyright            : (C) 2000 by Eric Espie
    email                : torcs@free.fr
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

#include <cstdlib>
#include <cmath>

#include <tgf.h>
#include <track.h>

#include <robottools.h>

void
TrackLocal2Global(tTrkLocPos *p, float *X, float *Y)
{
    RtTrackLocal2Global(p, X, Y, TR_TORIGHT);
}


void
TrackGlobal2Local(tTrackSeg *segment, float X, float Y, tTrkLocPos *p, int type)
{
    RtTrackGlobal2Local(segment, X, Y, p, type);
}
/*
 * Function
 *	
 *
 * Description
 *	
 *
 * Parameters
 *	
 *
 * Return
 *	
 */

float
TrackHeightL(tTrkLocPos *p)
{
    return RtTrackHeightL(p);
}

float
TrackHeightG(tTrackSeg *seg, float x, float y)
{
    return RtTrackHeightG(seg, x,  y);
}

void
TrackSideNormal(tTrackSeg *seg, float x, float y, int side, glm::vec3 *norm)
{
    RtTrackSideNormalG(seg, x, y, side, norm);
}

void
TrackSurfaceNormal(tTrkLocPos *p, glm::vec3 *norm)
{
    RtTrackSurfaceNormalL(p, norm);
}

float
TrackSpline(float p0, float p1, float t0, float t1, float t)
{
    float t2, t3;
    float h0, h1, h2, h3;
    
    t2 = t * t;
    t3 = t * t2;
    h1 = 3 * t2 - 2 * t3;
    h0 = 1 - h1;
    h2 = t3 - 2 * t2 + t;
    h3 = t3 - t2;
    
    return h0 * p0 + h1 * p1 + h2 * t0 + h3 * t1;
}
