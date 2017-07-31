/***************************************************************************

    file                 : strategy.cpp
    created              : Wed Sep 22 15:32:21 CET 2004
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
 Very simple stategy sample implementation.
*/


#include "strategy.h"

const float SimpleStrategy::MAX_FUEL_PER_METER = 0.0008f; // [kg/m] fuel consumtion.
const float SimpleStrategy::MAX_FUEL_PER_SECOND = 2.7f/60.0f;	// [kg/s] fuel consumtion.

SimpleStrategy::SimpleStrategy() :
 fuelchecked(false),
 fuelperlap(0.0),
 lastpitfuel(0.0)
{
 PIT_DAMMAGE = 5500;
 pit_damage = is_pitting = 0;
}


SimpleStrategy::~SimpleStrategy()
{
 // Nothing so far.
}


// Trivial strategy: fill in as much fuel as required for the whole race, or if the tank is
// too small fill the tank completely.
void SimpleStrategy::setFuelAtRaceStart(tTrack* t, void **carParmHandle, tSituation *s)
{
 // Load and set parameters.
 float fuel = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, HYMIE_ATT_FUELPERSECOND, (char*) NULL, MAX_FUEL_PER_SECOND);
 expectedfuelpersecond = fuel;
 fuel = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, HYMIE_ATT_FUELPERLAP, (char*) NULL, t->length*MAX_FUEL_PER_METER);
 expectedfuelperlap = fuel;
 maxfuel = GfParmGetNum(*carParmHandle, SECT_CAR, PRM_TANK, (char*) NULL, 100.0);
 fuel *= (s->_totLaps + 1.0);
 fuel += expectedfuelpersecond * MAX(s->_totTime,0);
 lastfuel = MIN(fuel, maxfuel);
 if (s->_raceType == RM_TYPE_PRACTICE)
  lastfuel = maxfuel;
 float initfuel =  GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "InitFuel", (char*) NULL, lastfuel);
 imaxfuel =  GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "MaxFuel", (char*) NULL, lastfuel);
 if (initfuel > 0.0)
  lastfuel = MIN(initfuel, lastfuel);
 if (imaxfuel > 0.0)
  lastfuel = MIN(imaxfuel, lastfuel);

 GfParmSetNum(*carParmHandle, SECT_CAR, PRM_FUEL, (char*) NULL, lastfuel);
}


void SimpleStrategy::update(tCarElt* car, tSituation *s)
{
 // Fuel statistics update.
 int id = car->_trkPos.seg->id;
 // Range must include enough segments to be executed once guaranteed.
 if (id >= 0 && id < 5 && !fuelchecked) {
  if (car->race.laps > 1) {
   fuelperlap = MAX(fuelperlap, (lastfuel+lastpitfuel-car->priv.fuel));
  }
  lastfuel = car->priv.fuel;
  lastpitfuel = 0.0;
  fuelchecked = true;
 } else if (id > 5) {
  fuelchecked = false;
 }

 if (car->_raceCmd == RM_CMD_PIT_ASKED)
  setStrategy(STRATEGY_PASSIVE);
 else if (car->_remainingLaps-car->_lapsBehindLeader < 3 && car->_dammage < PIT_DAMMAGE * 0.75 && car->_pos > 1)
  setStrategy(STRATEGY_DESPERATE);
 else if (car->_dammage > 2500)
  setStrategy(STRATEGY_CAREFUL);
 else
  setStrategy(STRATEGY_NORMAL);
}

int SimpleStrategy::calcRepair(tCarElt* car, tSituation *s, Opponents *opp, int inpit)
{
 // find out what our lead over next car is.
 float lead = FLT_MAX;
 int pos = 1000;
 Opponent *O = NULL;
 //int dammage = MIN(car->_dammage, PIT_DAMMAGE);

 // which car is behind me, not including team members?
 //if (car->_dammage < PIT_DAMMAGE)
 {
  if (car->_state == RM_CAR_STATE_PIT && pit_damage)
  {
   if (car->_remainingLaps-car->_lapsBehindLeader > 40)
    return car->_dammage;
   return MIN(car->_dammage, pit_damage);
  }

  for (int i = 0; i < opp->getNOpponents(); i++)
  {
   Opponent *o = opp->getOpponentPtr() + i;
   if (o->getTeam() == TEAM_CONTROL)
    continue;
   if (o->car->_state >= RM_CAR_STATE_PIT) continue;

   if (o->car->_pos < pos && o->car->_pos > car->_pos)
   {
    if (inpit)
    {
     float mytime = (car->_distFromStartLine / o->getTrackPtr()->length) * car->_lastLapTime + (car->_laps - o->car->_laps) * car->_bestLapTime;
     float othertime = (o->car->_distFromStartLine / o->getTrackPtr()->length) * o->car->_bestLapTime;
     lead = mytime - othertime;
     if (lead < 25.0)
      // lets accept that this car is past us & calculate vs the next one
      continue;
    }

    // base damage repair off this car unless we find a better one
    O = o;
    pos = o->car->_pos;
   }
  }
  if (O)
  {
   // how far behind is it?
   float mytime = (car->_distFromStartLine / O->getTrackPtr()->length) * car->_lastLapTime + (car->_laps - O->car->_laps) * car->_bestLapTime;
   float othertime = (O->car->_distFromStartLine / O->getTrackPtr()->length) * O->car->_bestLapTime;

   lead = mytime - othertime;

   // how much damage is it safe to fix?
   int safe_damage = 0;
   if (car->_state == RM_CAR_STATE_PIT)
    lead -= 15.0f + ((O->getTrackPtr()->pits.len * O->getTrackPtr()->pits.nMaxPits) / 20.0) * 0.30;
   else
    lead -= 15.0f + (O->getTrackPtr()->pits.len * O->getTrackPtr()->pits.nMaxPits) / 20.0;

   if (pit_reason == REASON_NONE)
    lead -= 20.0f;

   if (lead > 10.0f)
    safe_damage = (int) (lead / 0.007);

   if (pit_reason == REASON_DAMAGE)
   {
    if (car->_remainingLaps-car->_lapsBehindLeader > 40)
     safe_damage = car->_dammage;
    else
     safe_damage = MIN(car->_dammage, safe_damage);
   }

   return MIN(car->_dammage, safe_damage);
  }
  else
   return car->_dammage;
 }
}


bool SimpleStrategy::needPitstop(tCarElt* car, tSituation *s, Opponents *opp)
{
 // Do we need to refuel?
 int laps = car->_remainingLaps-car->_lapsBehindLeader;
 int this_pit_dammage = PIT_DAMMAGE;

 if (laps > 0) 
 {
  float cmpfuel = (fuelperlap == 0.0) ? expectedfuelperlap : fuelperlap;
  if (s->_totTime > s->currentTime)
  {
	if (car->_laps > 2)
		laps += (int)ceil( ( s->_totTime - s->currentTime ) / car->_bestLapTime + 0.3f );
	// For the case car->laps <= 2, the old laps only makes a stronger constraight, so no read to calculate it
  }				
  if (car->_fuel < 1.5*cmpfuel &&
      car->_fuel < laps*cmpfuel)
  {
   is_pitting = 1;
   pit_reason = REASON_FUEL;
   return true;
  }
  else if (laps < 20)
   this_pit_dammage = MIN(8000, PIT_DAMMAGE + (20-laps)*200);
 }

 if (car->_dammage < 9000 && (laps <= 2 || strategy == STRATEGY_DESPERATE))
 {
  is_pitting = 0;
  return false;
 }

 if (car->_dammage < 3000)
 {
  is_pitting = 0;
  return false;
 }

 // Do we need to repair?
 if (car->_dammage > this_pit_dammage) 
 {
  is_pitting = 1;
  pit_reason = REASON_DAMAGE;
  return true;
 }

 // Can we safely repair a lesser amount of damage?
 int damage;
 pit_reason = REASON_NONE;
 if ((damage = calcRepair(car, s, opp, 0)) >= MIN(2800, this_pit_dammage))
 {
  if (car->_pos < 4)
  {
   // if there's a chance of overtaking an opponent that's
   // not far in front, avoid going in to fix optional damage.
   for (int i = 0; i < opp->getNOpponents(); i++)
   {
    Opponent *o = opp->getOpponentPtr() + i;
    if (o->car->_pos >= car->_pos) continue;
    if (o->getTeam() == TEAM_CONTROL) continue;

    if (o->getDistance() > 100.0 && o->car->_fuel > 7.0 && o->car->_dammage < car->_dammage + 1500) continue;
    return false;
   }
  }

  if (damage < MIN(car->_dammage, this_pit_dammage))
   return false;

  if (is_pitting)
   pit_damage = MIN(car->_dammage, MAX(pit_damage, damage));
  else
   pit_damage = MIN(car->_dammage, MIN(pit_damage, damage));
  is_pitting = 1;
  pit_reason = REASON_DAMAGE;
  return true;
 }

 is_pitting = 0;
 return false;
}


float SimpleStrategy::pitRefuel(tCarElt* car, tSituation *s)
{
 float fuel;
 float cmpfuel = (fuelperlap == 0.0) ? expectedfuelperlap : fuelperlap;
 int laps = car->_remainingLaps;
 if (s->_totTime > s->currentTime)
 {
 	if (car->_laps > 2)
		laps += ( s->_totTime - s->currentTime ) / car->_bestLapTime;
  	else
 	{
 		// It has a pit stop in the first two laps. Normally we don't want to refuel.
		// This will cause for at least sufficient fuel.
		laps += 1;
 	}
 }
 fuel = MAX(MIN((car->_remainingLaps+1.5)*cmpfuel - car->_fuel, car->_tank - car->_fuel), 0.0);
 imaxfuel =  GfParmGetNum(car->_carHandle, HYMIE_SECT_PRIV, "MaxFuel", (char*) NULL, fuel);
 if (imaxfuel > 0.0)
  fuel = MIN(imaxfuel, fuel);
 lastpitfuel = fuel;
 fprintf(stderr,"%s: refueling %.2f\n",car->_name,fuel);fflush(stderr);
 return fuel;
}


int SimpleStrategy::pitRepair(tCarElt* car, tSituation *s, Opponents *opp)
{
 int dammage = calcRepair(car, s, opp, 1); //car->_dammage;
 pit_damage = 0;
 if (car->_dammage > PIT_DAMMAGE || car->_remainingLaps-car->_lapsBehindLeader > 40)
 {
  float laps = car->_remainingLaps + 1.0f;
  if (car->_laps > 2 && s->_totTime > s->currentTime)
	laps += ceil( ( s->_totTime - s->currentTime ) / car->_bestLapTime + 0.3f );
  float cmpfuel = (fuelperlap == 0.0) ? expectedfuelperlap : fuelperlap;  
  float fuel = MAX(MIN(laps*cmpfuel - car->_fuel, car->_tank), 0.0);

  if (fuel < car->_tank-15.0)
   dammage = MIN(car->_dammage, dammage);
  else
   dammage = car->_dammage;
 }
 else
  dammage = MIN(car->_dammage, dammage);
 fprintf(stderr,"%s: repairing %d damage\n",car->_name,dammage);fflush(stderr);
 return dammage;
}


