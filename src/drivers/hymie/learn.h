/***************************************************************************

    file                 : learn.h
    created              : Wed Aug 28 16:36:00 CET 2004
    copyright            : (C) 2004 by Bernhard Wymann
    email                : berniw@bluewin.ch
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


#ifndef _SEGLEARN_H_
#define _SEGLEARN_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <tgf.h>
#include <track.h>
#include <car.h>
#include <raceman.h>
#include <robottools.h>
#include <robot.h>



class SegLearn {
 public:
  SegLearn(tTrack* t, double l, int limit);
  ~SegLearn();

  double getBaseRadius(tTrackSeg *s, int avoiding) { return (avoiding ? baseradius[s->id] * 0.95 : baseradius[s->id]); }
  double getRadius(tTrackSeg *s, int avoiding) { return (avoiding ? avoidradius[s->id] : radius[s->id]); }
	void incrBaseRadius(tTrackSeg *s, double incr) { baseradius[s->id] = MAX(baseradius[s->id], incr); }
  void update(tSituation *s, tTrack *t, tCarElt *car, int alone, int doing_stuff, double outside, double *r);
  void incrRadius(tTrackSeg *s, double incr);

 private:
  tTrackSeg *lastseg;
  double *baseradius;
  double *radius;
  double *avoidradius;
  double *minradius;
  double *avoidminradius;
  int *learncount;
  int *updateid;

  double lastrmin;
  double learnfactor;
  int learnlimit;
  bool check;
  double rmin;
  int lastturn;
  int prevtype;
  int last_inside_error;
  int last_dammage;
};


#endif //_SEGLEARN_H_

