/***************************************************************************

    file                 : pit.cpp
    created              : Thu Mai 15 2:43:00 CET 2003
    copyright            : (C) 2003-2004 by Bernhard Wymann
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


#include "pit.h"

const float Pit::SPEED_LIMIT_MARGIN = 0.5;  // [m/s] savety margin to avoid pit speeding.


Pit::Pit(tSituation *s, Driver *driver)
{
 track = driver->getTrackPtr();
 car = driver->getCarPtr();
 mypit = driver->getCarPtr()->_pit;
 pdriver = driver;
 pitinfo = &track->pits;
 pitstop = inpitlane = lastinpit = false;
 pittimer = 0.0;

 if (mypit != NULL) {
  speedlimit = pitinfo->speedLimit - SPEED_LIMIT_MARGIN;
  speedlimitsqr = speedlimit*speedlimit;
  pitspeedlimitsqr = pitinfo->speedLimit*pitinfo->speedLimit;

  // Compute pit spline points along the track.
  p0[3].x = mypit->pos.seg->lgfromstart + mypit->pos.toStart;
  p1[3].x = mypit->pos.seg->lgfromstart + mypit->pos.toStart + pitinfo->len/4;
  p2[3].x = mypit->pos.seg->lgfromstart + mypit->pos.toStart - 2.0;
  p0[2].x = p1[2].x = p2[2].x = p0[3].x - pitinfo->len;
  p0[4].x = p1[4].x = p2[4].x = p0[3].x + pitinfo->len;
  p0[0].x = p1[0].x = p2[0].x = pitinfo->pitEntry->lgfromstart + driver->getPitEntryOffset();
  p0[1].x = p1[1].x = p2[1].x = pitinfo->pitStart->lgfromstart;
  p0[5].x = p1[5].x = p2[5].x = p0[3].x + (pitinfo->nMaxPits - car->index)*pitinfo->len;
  p0[6].x = p1[6].x = p2[6].x = pitinfo->pitExit->lgfromstart;

  pitentry = p0[0].x;
  pitapproach = pitentry - 200.0f;
  if (pitapproach < 0.0f)
   pitapproach += track->length;
  pitend = p0[5].x;
  pitexit = p0[6].x;

  // Normalizing spline segments to >= 0.0.
  int i;
  for (i = 0; i < NPOINTS; i++) {
   p0[i].s = p1[i].s = p2[i].s = 0.0;
   p0[i].x = toSplineCoord(p0[i].x);
   p1[i].x = toSplineCoord(p1[i].x);
   p2[i].x = toSplineCoord(p2[i].x);
  }

  // Fix broken pit exit.
  if (p0[6].x < p0[5].x) {
   //printf("bt: Pitexit broken on track %s.\n", track->name);
   p0[6].x = p0[5].x + 50.0;
   p1[6].x = p1[5].x + 50.0;
   p2[6].x = p2[5].x + 50.0;
  }

  // Fix point for first pit if necessary.
  if (p0[1].x > p0[2].x) {
   p0[1].x = p0[2].x;
   p1[1].x = p1[2].x;
   p2[1].x = p2[2].x;
  }

  // Fix point for last pit if necessary.
  if (p0[4].x > p0[5].x) {
   p0[5].x = p0[4].x;
   p1[5].x = p1[4].x;
   p2[5].x = p2[4].x;
  }

  float sign = (pitinfo->side == TR_LFT) ? 1.0 : -1.0;
  p0[0].y = p1[0].y = p2[0].y = 0.0;
  p0[6].y = p1[6].y = p2[6].y = 0.0;
  for (i = 1; i < NPOINTS - 1; i++) {
   p0[i].y = fabs(pitinfo->driversPits->pos.toMiddle) - pitinfo->width;
   p0[i].y *= sign;
   p1[i].y = fabs(pitinfo->driversPits->pos.toMiddle) - pitinfo->width;
   p1[i].y *= sign;
   p2[i].y = fabs(pitinfo->driversPits->pos.toMiddle) - pitinfo->width;
   p2[i].y *= sign;
  }

  p0[3].y = p1[3].y = p2[3].y = (fabs(pitinfo->driversPits->pos.toMiddle) + 0.5)*sign;
  spline0 = new Spline(NPOINTS, p0);
  spline1 = new Spline(NPOINTS, p1);
  spline2 = new Spline(NPOINTS, p2);
 }
}


Pit::~Pit()
{
 if (mypit != NULL) {
  delete spline0;
  delete spline1;
  delete spline2;
 }
}


// Transforms track coordinates to spline parameter coordinates.
float Pit::toSplineCoord(float x)
{
 x -= pitentry;
 while (x < 0.0) {
  x += track->length;
 }
 return x;
}

#define RELAX(target, prev, rate) {(target) = (prev) + rate * ((target) - (prev)) * (float) RCM_MAX_DT_ROBOTS; (prev) = (target);}

// Computes offset to track middle for trajectory.
double Pit::getPitOffset(double offset, double fromstart, double nextleft, int which)
{
 lastinpit = false;
 if (mypit != NULL) {
  if (getInPit() || (getPitstop() && isBetween(fromstart))) {
   if (getPitstop()) 
  {
  if (!lastinpit)
  {
   p0[0].y = p1[0].y = p2[0].y = car->_trkPos.toMiddle;
   delete spline0;
   delete spline1;
   delete spline2;
   spline0 = new Spline(NPOINTS, p0);
   spline1 = new Spline(NPOINTS, p1);
   spline2 = new Spline(NPOINTS, p2);
  }

  lastinpit = true;
  }
  fromstart = toSplineCoord(fromstart);
  double newoffset;
  if (which == 1)
   newoffset = spline1->evaluate(fromstart);
  else if (which == 2)
   newoffset = spline2->evaluate(fromstart);
  else
   newoffset = spline0->evaluate(fromstart);

  if (car->_distFromStartLine > p0[0].x && 0)
  {
   double incfactor = (5.0 - MIN(fabs(car->_speed_x*0.5)/(5.0), (5.0-1.0))) * 3;
   double nextoffset = car->_trkPos.seg->width * 0.5 - nextleft;

   if (newoffset > nextoffset)
    newoffset = MIN(newoffset, nextoffset + incfactor);
   else
    newoffset = MAX(newoffset, nextoffset - incfactor);
  }

  if (isExiting())
  {
   pdriver->setCorrecting(1);
#if 0
    switch(track->pits.side)
    {
     case TR_LFT:
      if (newoffset > track->width * 0.5 - 2.0f)
       newoffset -= 14.0 * (float) RCM_MAX_DT_ROBOTS;
      break;
     case TR_RGT:
      if (newoffset < -track->width * 0.5 + 2.0f)
       newoffset += 14.0 * (float) RCM_MAX_DT_ROBOTS;
      break;
    }
#endif
   }

   return newoffset;
  }
 }
 return offset;
}


// Sets the pitstop flag if we are not in the pit range.
void Pit::setPitstop(bool pitstop)
{
 if (mypit == NULL) {
  return;
 }

 float fromstart = car->_distFromStartLine;

 if (!isBetween(fromstart)) {
  this->pitstop = pitstop;
 } else if (!pitstop) {
  this->pitstop = pitstop;
  pittimer = 0.0;
 }
}


// Check if the argument fromstart is in the range of the pit.
bool Pit::isBetween(float fromstart)
{
 float end = pitexit;
 if (pitentry <= end) {
  if (fromstart >= pitentry && fromstart <= end) {
   return true;
  } else {
   return false;
  }
 } else {
  // Warning: TORCS reports sometimes negative values for "fromstart"!
  if (fromstart <= end || fromstart >= pitentry) {
   return true;
  } else {
   return false;
  }
 }
}

// Check if the argument fromstart is in the range of the pit.
bool Pit::isApproaching(float fromstart)
{
 if (car->_raceCmd != RM_CMD_PIT_ASKED || lastinpit || getInPit() || isBetween(fromstart))
  return false;
 if (pitapproach <= pitentry) {
  if (fromstart >= pitapproach) {
   return true;
  } else {
   return false;
  }
 } else {
  // Warning: TORCS reports sometimes negative values for "fromstart"!
  if (fromstart <= pitentry || fromstart >= pitapproach) {
   return true;
  } else {
   return false;
  }
 }
}

bool Pit::isApproaching() {
 return isApproaching(car->_distFromStartLine);
}

// Check if the argument fromstart is in the range of the pit.
bool Pit::isExiting()
{
 float fromstart = car->_distFromStartLine;
 if ((track->pits.side == TR_LFT && car->_trkPos.toLeft > 0.0) ||
     (track->pits.side == TR_RGT && car->_trkPos.toRight > 0.0))
  return false;

 if (pitend <= pitexit) {
  if (fromstart >= pitend && fromstart <= pitexit) {
   return true;
  } else {
   return false;
  }
 } else {
  // Warning: TORCS reports sometimes negative values for "fromstart"!
  if (fromstart <= pitexit || fromstart >= pitend) {
   return true;
  } else {
   return false;
  }
 }
}


// Checks if we stay too long without getting captured by the pit.
// Distance is the distance to the pit along the track, when the pit is
// ahead it is > 0, if we overshoot the pit it is < 0.
bool Pit::isTimeout(float distance)
{
 if (car->_speed_x > 1.0 || distance > 3.0 || !getPitstop()) {
  pittimer = 0.0;
  return false;
 } else {
  pittimer += (float) RCM_MAX_DT_ROBOTS;
  if (pittimer > 3.0) {
   pittimer = 0.0;
   return true;
  } else {
   return false;
  }
 }
}


// Update pit data and strategy.
void Pit::update()
{
 if (mypit != NULL) {
  if (isBetween(car->_distFromStartLine)) {
   if (getPitstop()) {
    setInPit(true);
   }
  else if (car->_trkPos.toLeft > 0.0 && car->_trkPos.toRight > 0.0)
  setInPit(false);
  } else {
   setInPit(false);
  }

  if (getPitstop()) {
   car->_raceCmd = RM_CMD_PIT_ASKED;
  }
 }
}


float Pit::getSpeedLimitBrake(float speedsqr)
{
 return (speedsqr-speedlimitsqr)/(pitspeedlimitsqr-speedlimitsqr);
}

