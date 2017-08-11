/***************************************************************************

    file                 : trackinc.h
    created              : Sun Jan 30 22:57:40 CET 2000
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

#ifndef _TRACKINC_H__
#define _TRACKINC_H__

#include <track.h>

extern tTrack *TrackBuildv1(const char *trackfile);
extern tTrack *TrackBuildEx(const char *trackfile);
extern void TrackShutdown(void);

extern void ReadTrack3(tTrack *theTrack, void *TrackHandle, tRoadCam **camList, int ext);
extern void ReadTrack4(tTrack *theTrack, void *TrackHandle, tRoadCam **camList, int ext);
extern void ReadTrack5(tTrack *theTrack, void *TrackHandle, tRoadCam **camList, int ext);

extern tRoadCam *TrackGetCamList(void);

extern float TrackHeightG(tTrackSeg *seg, float x, float y);
extern float TrackHeightL(tTrkLocPos *p);
extern void TrackGlobal2Local(tTrackSeg *segment, float X, float Y, tTrkLocPos *p, int sides);
extern void TrackLocal2Global(tTrkLocPos *p, float *X, float *Y);
extern void TrackSideNormal(tTrackSeg*, float, float, int, glm::vec3*);
extern void TrackSurfaceNormal(tTrkLocPos *p, glm::vec3 *norm);
extern float TrackSpline(float p0, float p1, float t0, float t1, float t);


#endif /* _TRACKINC_H__ */ 



