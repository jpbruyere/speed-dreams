/***************************************************************************

    file                 : learn.cpp
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


#include <stdio.h>
#include "learn.h"

#ifdef WIN32
#define snprintf _snprintf
#endif

SegLearn::SegLearn(tTrack* t, double l, int limit)
{
 learnlimit = limit;
 if (!learnlimit)
  learnlimit = 100000;

 int i;
 baseradius = new double[t->nseg];
 radius = new double[t->nseg];
 minradius = new double[t->nseg];
 avoidradius = new double[t->nseg];
 avoidminradius = new double[t->nseg];
 updateid = new int[t->nseg];
 learncount = new int[t->nseg];
 tTrackSeg *seg = t->seg;

 learnfactor = l;

 // Switch seg to seg 0 for sure.
 while (seg->id != 0) {
  seg = seg->prev;
 }

 for (i = 0; i < t->nseg; i++) {
  radius[i] = baseradius[i] = 0.0;
  minradius[i] = 10000.0;
  avoidradius[i] = 0.0;
  avoidminradius[i] = 10000.0;
  updateid[i] = i;
  learncount[i] = 0;
  // Search the last turn in case of a straight.
  if (seg->type == TR_STR) {
   tTrackSeg *cs = seg;
   while (cs->type == TR_STR) {
    cs = cs->prev;
   }
   updateid[seg->id] = cs->id;
  }
  seg = seg->next;
 }

 check = false;
 rmin = t->width/2.0;
 prevtype = lastturn = TR_STR;
 last_inside_error = last_dammage = 0;
 lastrmin = 0.0;
 lastseg = NULL;
}


SegLearn::~SegLearn()
{
 delete [] baseradius;
 delete [] radius;
 delete [] minradius;
 delete [] avoidradius;
 delete [] avoidminradius;
 delete [] updateid;
 delete [] learncount;
}


void SegLearn::update(tSituation *s, tTrack *t, tCarElt *car, int alone, int avoiding, double outside, double *r)
{
 // Still on the same segment, alone, offset near 0, check.
 tTrackSeg *seg = car->_trkPos.seg;
 tTrackSeg *tseg = seg;

 if (seg->type == lastturn || seg->type == TR_STR) 
 {
  if (check == true && alone > 0) 
  {
   // + to left, - to right
   double toleft = car->_trkPos.toLeft;
   double dr = 0.0;
   int inside_error = 0;

   if (lastturn == TR_RGT) {
    dr = toleft - MAX(MIN(outside, 1.5), outside - 1.5);
    if (car->_trkPos.toRight < car->_dimension_y/4)
     inside_error = 1;
   } else if (lastturn == TR_LFT) {
    dr = MIN(MAX(outside, seg->width-1.5), outside + 1.5) - toleft;
    if (car->_trkPos.toLeft < car->_dimension_y/4)
     inside_error = 1;
   }

   // decrease speed further if we're spinning out or hit a wall
   int spindmg_error = 0;
   double angle = RtTrackSideTgAngleL(&(car->_trkPos));
   angle -= car->_yaw;
   NORM_PI_PI(angle);
   if (!avoiding)
   {
    if (fabs(angle) > 1.3 && fabs(car->_yaw_rate) > 1.5)
    {
     dr = MIN(0.0, dr) - (fabs(angle) + fabs(car->_yaw_rate))*2;
     spindmg_error = 1;
    }
    else if (car->_dammage > last_dammage + 500)
    {
     dr = MIN(dr, -((double)(car->_dammage-last_dammage)/500.0));
     spindmg_error = 1;
    }
   }
   
   // this stops it 'learning' to over-accelerate though the first 
   // corner of a chicane.
   if ((!avoiding && seg->radius <= 50.0 &&
      (seg->type == TR_RGT && car->_trkPos.toRight < car->_dimension_y/4)) ||
       (seg->type == TR_LFT && car->_trkPos.toLeft < car->_dimension_y/4))
   {
    inside_error = 1;
    tTrackSeg *cs = seg->prev;
    double len = 0.0;
 
    while (cs->type == seg->type && len < 100.0)
    {
     len += cs->length;
     cs = cs->prev;
    }
 
    if (cs->type != TR_STR && cs->type != seg->type && cs->radius < 400.0 && !last_inside_error)
    {
     int thisturn = cs->type;
     double redux = (seg->type == TR_RGT ? (car->_dimension_y/2 - car->_trkPos.toRight) : (car->_dimension_y/2 - car->_trkPos.toLeft));
     while (cs->type == thisturn)
     {
      if (radius[updateid[cs->id]] > 0.0 && learncount[updateid[cs->id]] < learnlimit)
      {
       minradius[updateid[cs->id]] = MIN(minradius[updateid[cs->id]], radius[updateid[cs->id]] - (redux/2));
       radius[updateid[cs->id]] -= redux;
       radius[updateid[cs->id]] = MIN(radius[updateid[cs->id]], 1000.0);
       radius[updateid[cs->id]] = MAX(radius[updateid[cs->id]], -cs->radius+1.0);
       learncount[updateid[cs->id]]++;
      }
      cs = cs->prev;
     }
     return;
    }
    else if (!last_inside_error)
     inside_error = 0;
   }
   if (dr < rmin && !inside_error) {
    rmin = dr;
   }
   if (rmin >= 0.0)
    rmin = MAX(rmin, 0.1);
   last_inside_error = inside_error;
  } else {
   check = false;
  }
 }

 if (seg->type != prevtype || seg != lastseg) {
  prevtype = seg->type;
  if (lastseg != seg)
  {
   lastseg = seg;
  }
  lastrmin = rmin;
  if (seg->type != TR_STR) {
   if (check == true) {
    tTrackSeg *cs = tseg->prev;
    // Skip straights.
    while (cs->type == TR_STR) {
     cs = cs->prev;
    }

    while (cs->type == lastturn) {
     if (rmin < 0.0)
     {
      rmin *= learnfactor;
      rmin /= MAX(1, learncount[updateid[cs->id]]);
     }

     if (!avoiding && learncount[updateid[cs->id]] < learnlimit)
     {
      if (radius[updateid[cs->id]] + rmin < 0.0) {
       rmin = MAX(cs->radius - r[cs->id], rmin);
      }
      double thisrmin = (rmin > 0.0 ? rmin : (minradius[updateid[cs->id]] < 10000.0 ? rmin * 0.5 : rmin * 1.1));
      if (rmin < 0.0)
      {
       minradius[updateid[cs->id]] = MIN(minradius[updateid[cs->id]], radius[updateid[cs->id]] + rmin * 0.2);
       radius[updateid[cs->id]] = (radius[updateid[cs->id]] + thisrmin) / 2;
      }
      else
       radius[updateid[cs->id]] += thisrmin;
      radius[updateid[cs->id]] = MIN(radius[updateid[cs->id]], 1000.0);
      radius[updateid[cs->id]] = MIN(radius[updateid[cs->id]], minradius[updateid[cs->id]]);
      avoidradius[updateid[cs->id]] = MIN(avoidradius[updateid[cs->id]], radius[updateid[cs->id]]);
      avoidminradius[updateid[cs->id]] = MIN(avoidminradius[updateid[cs->id]], minradius[updateid[cs->id]]);
      learncount[updateid[cs->id]]++;
     }
     else if (!avoiding)
     {
      if (avoidradius[updateid[cs->id]] + rmin < 0.0) {
       rmin = MAX(cs->radius - r[cs->id], rmin);
      }
      double thisrmin = (rmin < 0.0 ? rmin : (avoidminradius[updateid[cs->id]] < 10000.0 ? rmin * 0.5 : rmin * 1.5));
      if (rmin < 0.0)
      {
       avoidminradius[updateid[cs->id]] = MIN(avoidradius[updateid[cs->id]], avoidradius[updateid[cs->id]] + rmin * 0.2);
       avoidradius[updateid[cs->id]] = (avoidradius[updateid[cs->id]]+thisrmin) / 2;
      }
      else
       avoidradius[updateid[cs->id]] += thisrmin;
      avoidradius[updateid[cs->id]] = MIN(avoidradius[updateid[cs->id]], 1000.0);
      avoidradius[updateid[cs->id]] = MIN(avoidradius[updateid[cs->id]], avoidminradius[updateid[cs->id]]);
     }
     cs = cs->prev;
    }
   }
   check = true;
   rmin = MIN(seg->width/2.0, seg->radius/10.0);
   lastturn = seg->type;
  }
 }

 last_dammage = car->_dammage;
}

