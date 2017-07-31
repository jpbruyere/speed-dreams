/***************************************************************************

    file                 : strategy.h
    created              : Wed Sep 22 15:31:51 CET 2004
    copyright            : (C) 2004 Bernhard Wymann
    email                : berniw@bluewin.ch
    version              : $Id: strategy.h,v 1.3 2006/03/06 22:43:50 berniw Exp $

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
#include <teammanager.h>

#include "Driver.h"

class Opponents;

class AbstractStrategy
{
protected:
    enum
    {
        PIT_NONE,
        PIT_BEFORE,
        PIT_PREPARE,
        PIT_ENTER,
        PIT_ASKED,
        PIT_SERVICE,
        PIT_EXIT_WAIT,
        PIT_EXIT,
        PIT_GONE
    };
    int     m_State;		                           // Current pitting state
    bool    m_GoToPit;                               // Pit stop needed

    int   teamIndex;
    bool  releasePit;

public:
    AbstractStrategy()
      :m_State(PIT_NONE), m_GoToPit(false), Car(NULL), Track(NULL), pitPath(NULL), Driver(NULL), DistToSwitch(100), StartFuel(-1){};
    // Need this empty constructor... do not remove.
    virtual ~AbstractStrategy() {}
    // Set Initial fuel at race start.
    virtual void setFuelAtRaceStart(tTrack* t, void **carParmHandle, tSituation *s, int index) = 0;
    // Update internal data at every timestep.
    virtual void update(tCarElt* car, tSituation *s) = 0;
    // Do we need a pit stop? Can be called less frequently.
    virtual bool needPitstop(tCarElt* car, tSituation *s) = 0;
    // How much to refuel at pit stop.
    virtual float pitRefuel(tCarElt* car, tSituation *s) = 0;
    // How much repair at pit stop.
    virtual int pitRepair(tCarElt* car, tSituation *s) = 0;
    // Pit Free?
    virtual bool isPitFree(tCarElt* car) = 0;
    virtual int pitStopPenalty() = 0;

    tCarElt    *Car;
    tTrack     *Track;
    PitPath    *pitPath;
    TDriver    *Driver;
    //Pit     Pit;
    //TDriver   Driver;

    double    DistToSwitch;                           // Dist to Pit
    double    StartFuel;                              // Fuel at start
    double    MaxFuel;
    double    fullfuel;
    double    FuelperMeters;
    double    Reserve;
    double    TireLimitFront;
    double    TireLimitRear;
    double    DegradationPerLap;
    int       Laps;

    bool      OutOfPitlane(){return m_State < PIT_ENTER;}
};

class SimpleStrategy : public AbstractStrategy
{
public:
    SimpleStrategy();
    ~SimpleStrategy();

    void    setFuelAtRaceStart(tTrack* t, void **carParmHandle, tSituation *s, int index);
    void    update(tCarElt* car, tSituation *s);
    bool    needPitstop(tCarElt* car, tSituation *s);
    float   pitRefuel(tCarElt* car, tSituation *s);
    int     pitRepair(tCarElt* car, tSituation *s);
    bool    isPitFree(tCarElt* car);
    int     pitStopPenalty() { return m_stopPenalty; }

protected:
    bool test_Pitstop;
    bool test_qualifTime;
    bool fuelChecked;               // Fuel statistics updated.
    bool quickPitstop;              // Fast repair
    bool needRepair;                // Repair needed
    bool shortTrack;                // Set strategy for short track
    bool qualifRace;
    bool m_checkDamage;
    bool m_checkFuel;
    float fuelPerMeter;             // fuel consumtion.
    float fuelPerLap;               // The maximum amount of fuel we needed for a lap.
    float lastPitFuel;              // Amount refueled, special case when we refuel.
    float lastFuel;                 // The fuel available when we cross the start lane.
    float fuelSum;        			// All the fuel used needed for the race.
    int counterFuelLaps;     		// Counter of the total laps.
    int countPitStop;               // Counter of the total pits stop.
    double avgFuelPerLap;    		// The average amount of fuel we needed for a lap.
    int m_maxDamage;                // The Max damage set in the XML file.
    float m_Fuel;                   // The new average fuel per lap
    float m_expectedfuelperlap;		// Expected fuel per lap (may be very inaccurate).
    double maxFuel;
    float m_FuelStart;
    double m_lastpitfuel;
    int m_remainingstops;
    float calcFuel(double totalFuel);
    double getRefuel1(int laps);		// How much fuel it takes to reach the finish line.
    double getRefuel2(int laps);

    float MAX_FUEL_PER_METER;           // max fuel consumtion.
    double MAX_FUEL_TANK;               // max fuel for this car(fuel tank capacity).
    int PIT_DAMMAGE;                    // max damage before we request a pit stop. (M*2? Do not disturb!)
    int MAX_DAMAGE;
    int MAX_DAMAGE_DIST;

    int m_stopPenalty;

    int fuel_Strat;                     // Pit refuel strategy
    float m_fuelperstint;
    float m_pittime;                    // Expected additional time for pit stop.
    int AlwaysPit;
    int strategy_verbose;

	float	  TrackLength;
	float	  RaceDistance;

    int laps_to_go(tCarElt *car) {return car->_remainingLaps - car->_lapsBehindLeader;}
    virtual void updateFuelStrategy(tCarElt* car, tSituation *s);
};

#endif // _STRATEGY_H_
