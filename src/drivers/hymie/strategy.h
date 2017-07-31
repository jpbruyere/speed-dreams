/***************************************************************************

    file                 : strategy.h
    created              : Wed Sep 22 15:31:51 CET 2004
    copyright            : (C) 2004 Bernhard Wymann
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

/*
 Pit strategy for drivers. It defines an abstract base class, such that one can easily plug in
 different strategies.
*/

#ifndef _STRATEGY_H_
#define _STRATEGY_H_

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

#include "driver.h"

enum { STRATEGY_DESPERATE=0, STRATEGY_NORMAL, STRATEGY_CAREFUL, STRATEGY_PASSIVE };
enum { REASON_NONE=0, REASON_DAMAGE=1, REASON_FUEL=2 };

class Opponents;

class AbstractStrategy {
 public:
  // Need this empty constructor... do not remove.
  virtual ~AbstractStrategy() {}
  // Set Initial fuel at race start.
  virtual void setFuelAtRaceStart(tTrack* t, void **carParmHandle, tSituation *s) = 0;
  // Update internal data at every timestep.
  virtual void update(tCarElt* car, tSituation *s) = 0;
  // Do we need a pit stop? Can be called less frequently.
  virtual bool needPitstop(tCarElt* car, tSituation *s, Opponents *opp) = 0;
  // How much to refuel at pit stop.
  virtual float pitRefuel(tCarElt* car, tSituation *s) = 0;
  // How much repair at pit stop.
  virtual int pitRepair(tCarElt* car, tSituation *s, Opponents *opp) = 0;
  virtual int getStrategy() { return strategy; }
  virtual float getMaxFuel() { return maxfuel; }
  virtual void setStrategy(int strat) { strategy = strat; }
  virtual void setPitDamage(int damage) = 0;

 protected:
  int strategy;
  float maxfuel;
};


class SimpleStrategy : public AbstractStrategy {
 public:
  SimpleStrategy();
  ~SimpleStrategy();

  void setFuelAtRaceStart(tTrack* t, void **carParmHandle, tSituation *s);
  void update(tCarElt* car, tSituation *s);
  bool needPitstop(tCarElt* car, tSituation *s, Opponents *opp);
  float pitRefuel(tCarElt* car, tSituation *s);
  int pitRepair(tCarElt* car, tSituation *s, Opponents *opp);
  void setPitDamage(int damage) { PIT_DAMMAGE = damage; }
  void setMinDamage(int damage) { min_damage = damage; }

 protected:
  int calcRepair(tCarElt *car, tSituation *s, Opponents *opp, int inpit);
  bool fuelchecked;    // Fuel statistics updated.
  float fuelperlap;    // The maximum amount of fuel we needed for a lap.
  float lastpitfuel;    // Amount refueled, special case when we refuel.
  float lastfuel;     // the fuel available when we cross the start lane.
  float expectedfuelperlap;  // Expected fuel per lap (may be very inaccurate).
  float expectedfuelpersecond;		// Expected fuel per second (may be very inaccurate).
  float imaxfuel;

  static const float MAX_FUEL_PER_METER; // [kg/m] fuel consumtion.
  static const float MAX_FUEL_PER_SECOND; // [kg/s] fuel consumtion.
  int PIT_DAMMAGE;   // If damage > we request a pit stop.
  int pit_damage;
  int min_damage;
  int is_pitting;
  int remainlaps;
  int pit_reason;
};



#endif // _STRATEGY_H_


