/***************************************************************************

    file                 : opponent.h
    created              : Thu Apr 22 01:20:19 CET 2003
    copyright            : (C) 2007 Andrew Sumner, 2003-2004 Bernhard Wymann
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

#ifndef _OPPONENT_H_
#define _OPPONENT_H_

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

#include "linalg.h"
#include "driver.h"
#include "cardata.h"

#define OPP_IGNORE  0
#define OPP_FRONT  (1<<0)
#define OPP_BACK  (1<<1)
#define OPP_SIDE  (1<<2)
#define OPP_COLL  (1<<3)
#define OPP_LETPASS  (1<<4)
#define OPP_FRONT_FAST (1<<5)
#define OPP_FOLLOW  (1<<6)
#define OPP_COLL_URGENT  (1<<7)
#define OPP_SIDE_FRONT (1<<8)
#define OPP_SIDE_COLL (1<<9)

enum { CLOSING = 1, CLOSING_FAST = 2 };


class Driver;

// Opponent maintains the data for one opponent RELATIVE to the drivers car.
class Opponent {
 public:
  Opponent();

  void setCarPtr(tCarElt *car) { this->car = car; prevleft = car->_trkPos.toLeft; }
  void setCarDataPtr(SingleCardata *cardata) { this->cardata = cardata; }
  static void setTrackPtr(tTrack *track) { Opponent::track = track; }
  tTrack *getTrackPtr() { return track; }

  tCarElt *getCarPtr() { return car; }
  int getState() { return state; }
  void setState(int newstate) { state |= newstate; }
  double getCatchDist() { return catchdist; }
  double getDistance() { return distance; }
  double getBrakeDistance() { return brakedistance; }
  double getNextDistance() { return nextdistance; }
  double getCloseDist() { return closedistance; }
  double getWidth() { return cardata->getWidthOnTrack(); }
  double getNextWidth() { return nextwidth; }
  double getSpeed() { return cardata->getSpeedInTrackDirection(); }
  double getNextSpeed() { return nextspeed; }
  double getOverlapTimer() { return overlaptimer; }
  double getNextLeft() { return nextleft; }
  double getNextLeft2() { return nextleft + (nextleft2 - nextleft) * 0.3; }
  double getTimeImpact() { return t_impact; }
  double getMaxTimeImpact() { return MAX(t_impact, s_impact); }
  double getBrakeSpeed() { return brake_speed; }
  double getAngle() { return angle; }
  double getLOffset() { return Loffset; }
  double getLastLOffset() { return lastLoffset; }
  double getROffset() { return Roffset; }
  double getLastROffset() { return lastRoffset; }
  int getTeam() { return team; }
  int getClosing() { return closing; }

  void update(tSituation *s, Driver *driver);
  tCarElt *car;

 private:
  float getDistToSegStart();
  double testCollision(tPosd *corner0, tPosd *corner1, tPosd *corner2, int point_test, tPosd *colpoint);
  void updateOverlapTimer(tSituation *s, tCarElt *mycar);

  double distance;  // approximation of the real distance, negative if the opponent is behind.
  double catchdist; // distance needed to catch the opponent (linear estimate).
  double sidedist;  // approx distance of center of gravity of the cars.
  double sidegap, prevsidegap; 
  double overlaptimer;
  double lastspeed;
  double nextspeed;
  double prevtime;
  double prevleft;
  double nextleft;
  double nextleft2;
  double prevwidth;
  double nextwidth;
  double t_impact;
  double s_impact;
  double brake_speed;
  double cardist;
  double angle;
  double Loffset, Roffset;
  double lastLoffset, lastRoffset;
  double nextLoffset, nextRoffset;
  double nextdistance, prevdistance, brakedistance, closedistance;

  tPosd corner1[4];
  tPosd corner2[4];

  int team;
  int state;   // State variable to characterize the relation to the opponent, e. g. opponent is behind.
  int closing;

  SingleCardata *cardata;  // Pointer to global data about this opponent.

  // class variables.
  static tTrack *track;

  // constants.
  static const float FRONTCOLLDIST;
  static const float BACKCOLLDIST;
  static const float LENGTH_MARGIN;
  static const float SIDE_MARGIN;
  static const float TIME_MARGIN;
  static const float EXACT_DIST;
  static const float LAP_BACK_TIME_PENALTY;
  static const float OVERLAP_WAIT_TIME;
  static const float SPEED_PASS_MARGIN;
};


// The Opponents class holds an array of all Opponents.
class Opponents {
 public:
  Opponents(tSituation *s, Driver *driver, Cardata *cardata);
  ~Opponents();

  void update(tSituation *s, Driver *driver);
  Opponent *getOpponentPtr() { return opponent; }
  int getNOpponents() { return nopponents; }

 private:
  Opponent *opponent;
  int nopponents;
};


#endif // _OPPONENT_H_
