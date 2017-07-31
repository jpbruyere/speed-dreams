/***************************************************************************

    file                 : strategy.cpp
    created              : Wed Sep 22 15:32:21 CET 2004
    copyright            : (C) 2004 Bernhard Wymann
    email                : berniw@bluewin.ch
    version              : $Id: strategy.cpp,v 1.3 2006/03/06 22:43:50 berniw Exp $

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

//#define STRAT_DEBUG

#include "Strategy.h"


SimpleStrategy::SimpleStrategy()
{
    MAX_FUEL_PER_METER = 0.00068f;		// [liter/m] max fuel consumtion.
    MAX_FUEL_TANK = 83.0f;              // [Kg] Fuel: Car Tank capacity.(for car3-trb1)
    PIT_DAMMAGE = 5000;                 // [-] max damage before test_Pitstop
    MAX_DAMAGE = 4000;              	// [-] max damage limit in the last five laps before the pit stop.
    fuel_Strat = 1;                     // [-] Pit refuel strategy for Short(1) or Long(2) track.

    needRepair = false;                 // [-] Need to repair (amount of damage)
    quickPitstop = false;               // [-] Fast repair (30% of damage)
    fuelChecked = false;                // [-] Fuel statistics updated.
    shortTrack = false;                 // [-] Strategy for short track enable/disable.
    qualifRace = false;                 // [-] Qualifying race type enable.
    m_checkDamage = false;              // [-] Dammage to repair checked.
    m_checkFuel = false;                // [-] Needed fuel checked.
    strategy_verbose = 0;               // [-] Display information about refuel and repair.

    FuelperMeters = 0.0f;
    fuelPerLap = 0.0f;                  // [Kg] The maximum amount of fuel we needed for a lap.
    lastPitFuel = 0.0f;                 // [Kg] Amount refueled, special case when we refuel.
    fuelSum = 0.0f;                     // [Kg] All the fuel used.
    counterFuelLaps = 0;                // [-] Counter of the total laps.
    countPitStop = 0;                   // [-] Counter of the total pitStop.
    avgFuelPerLap = 0.0;                // [Kg] Average fuel per lap.
    m_maxDamage = 0;                    // [-] max damage before we request a pit stop.
    m_Fuel = 0;                         // [Kg] Security fuel at the strat race.
    m_expectedfuelperlap = 0;           // [Kg] Expected fuel per lap
	TrackLength = 0;
	RaceDistance = 0;
}


SimpleStrategy::~SimpleStrategy()
{
    // Nothing so far.
}


void SimpleStrategy::setFuelAtRaceStart(tTrack* t, void **carParmHandle, tSituation *s, int index)
{
	Track = t;
	TrackLength = Track->length;
    LogSHADOW.info("Strategy Track Lengh =  %.2f\n", TrackLength);
	RaceDistance = TrackLength * s->_totLaps; 
    LogSHADOW.info("Strategy Race Distance =  %.2f\n", RaceDistance);
    /* Trivial strategy: fill in as much fuel as required for the whole race, or if the tank is
       too small fill the tank completely. */
    // Load and set parameters.
    maxFuel = (tdble)(GfParmGetNum(*carParmHandle, SECT_CAR, PRM_TANK, (char*)NULL, (tdble) MAX_FUEL_TANK));
    LogSHADOW.info("Strategy max fuel =  %.2f\n", maxFuel);
    FuelperMeters = GfParmGetNum(*carParmHandle, SECT_PRIV, PRV_FUELPERMETERS, (char*)NULL, (tdble) MAX_FUEL_PER_METER);
    LogSHADOW.info("Strategy fuel per meters =  %.5f\n", FuelperMeters);
    fuelPerLap = (tdble) (TrackLength * FuelperMeters);
    LogSHADOW.info("Strategy fuel per lap =  %.2f\n", fuelPerLap);
    fullfuel = GfParmGetNum(*carParmHandle, SECT_PRIV, PRV_FULL_FUEL, (char*)NULL, 0.0);
    fuel_Strat = (int)GfParmGetNum(*carParmHandle, SECT_PRIV, PRV_PITSTRAT, (char*)NULL, 0.0);
    test_Pitstop = (GfParmGetNum(*carParmHandle, SECT_PRIV, PRV_PITSTOP, (char*)NULL, 0.0) != 0);
    test_qualifTime = (GfParmGetNum(*carParmHandle, SECT_PRIV, PRV_CHKQUALIF, (char*)NULL, 0.0) != 0);
    strategy_verbose = (int)GfParmGetNum(*carParmHandle, SECT_PRIV, PRV_VERBOSE, (char*)NULL, 0.0);

    if ( fuel_Strat < 1 )
    {
        fuel_Strat = 1 ;
    }

    if ( fuel_Strat == 1 )
    {
        shortTrack = true;
    }

    m_expectedfuelperlap = fuelPerLap;
    //double raceDist = 0.0;                // Removed 6th April 2015 - Not Used
    float fuelForRace = 0.0;
    int numPitstop = 0;
    int raceLaps = s->_totLaps + 1;
    //raceDist = raceLaps * t->length;
    //fuelForRace = raceDist * fuelPerMeter;
    fuelForRace = raceLaps * fuelPerLap;
    numPitstop = (int) fabs(fuelForRace / maxFuel);
    if ( numPitstop < fuelForRace / maxFuel )
    {
        numPitstop = numPitstop + 1;
    }

    double m_fuel; // + security fuel.

    if (shortTrack)
    {
        m_fuel = fuelPerLap * 1.2;
    } else
    {
        m_fuel = fuelPerLap * 1.3;
    }

    if ( index == 1 )
    {
        m_fuel = fuelPerLap;
    }

    // Initial fuel at Start race
    if (fuelForRace > maxFuel * 3)
    {
        // welcome in Endurance race :-)
        m_FuelStart = (tdble)(m_fuel + calcFuel(fuelForRace));

    } else if (fuelForRace < maxFuel)
    {
        //maybe we are in Practice mode
        m_FuelStart = (tdble)(m_fuel + raceLaps * fuelPerLap);
    }
    else
    {
        m_FuelStart = (tdble)(m_fuel + fuelForRace);
    }

    if (s->_raceType == RM_TYPE_QUALIF)
    {
        qualifRace = true;
        m_fuel = 0;
        if (index == 1)
        {
            m_FuelStart = (tdble)((s->_totLaps * fuelPerLap) + 0.0);
        } else
        {
            m_FuelStart = s->_totLaps * fuelPerLap;
        }
    }
    /* Tests in Practice mode */
    //maybe we need to check PitStop (eg. corkscrew, e-track-3)
    if (test_Pitstop && s->_raceType == RM_TYPE_PRACTICE)
    {
        m_fuel = 0;
        //m_FuelStart = ((raceLaps / 2) * fuelPerLap) + 0.3;
        m_FuelStart = (tdble)(1.92 * fuelPerLap);
        fprintf(stderr,"...Check PitStop Enabled!\n");
        //maybe we need to check Best Lap Time for qualifying
    } else if (test_qualifTime && s->_raceType == RM_TYPE_PRACTICE)
    {
        qualifRace = true;
        m_fuel = 0;
        m_FuelStart = s->_totLaps * fuelPerLap;
        fprintf(stderr,"...Check QualifTime Enabled!\n");
    }

    if (m_FuelStart > maxFuel)
    {
        m_FuelStart = (tdble) maxFuel;
        if (index == 1)
        {
            m_FuelStart = (tdble)(maxFuel - fuelPerLap);
        }
    }

    if (fullfuel > 0)
        m_FuelStart = (tdble) fullfuel;

    m_FuelStart = (tdble)(MIN(m_FuelStart, maxFuel));

    fprintf(stderr,"# SHADOW Index %d : Laps = %d, Fuel per Lap = %.2f, securityFuel = + %.2f, Fuel at Start Race = %.2f\n",
            index, s->_totLaps, fuelPerLap, m_fuel, m_FuelStart);

    GfParmSetNum(*carParmHandle, SECT_CAR, PRM_FUEL, (char*)NULL, m_FuelStart);
}


void SimpleStrategy::update(tCarElt* car, tSituation *s)
{
    // Fuel statistics update.
    int id = car->_trkPos.seg->id;
    /* Range must include enough segments to be executed once guaranteed. */
    if (id >= 0 && id < 5 && !fuelChecked)
    {
        if (car->race.laps > 1)
        {
            //fuelPerLap = MAX(fuelPerLap, (lastFuel + lastPitFuel - car->priv.fuel));
            fuelSum += (lastFuel + lastPitFuel - car->priv.fuel);
            fuelPerLap = (fuelSum/(car->race.laps - 1));
            counterFuelLaps++;
            avgFuelPerLap = fuelSum / counterFuelLaps;

            if (strategy_verbose)
            {
                // Just display pit refuel messages
                updateFuelStrategy(car, s);
            }
        }

        lastFuel = car->priv.fuel;
        lastPitFuel = 0.0;
        fuelChecked = true;
    } else if (id > 5)
    {
        fuelChecked = false;
    }
}

void SimpleStrategy::updateFuelStrategy(tCarElt* car, tSituation *s)
{
    double requiredfuel;

    /* Required additional fuel for the rest of the race. +1 because
       the computation happens right after crossing the start line. */

    //requiredfuel = ((car->_remainingLaps + 1) - ceil(car->_fuel/fuelPerLap))*fuelPerLap;
    requiredfuel = ((car->_remainingLaps + 1) *fuelPerLap) - ceil(car->_fuel);

    if (!m_checkFuel && requiredfuel <= 0.0f && car->_remainingLaps <= 5)
    {
        // We have enough fuel to end the race, no further stop required.
        LogSHADOW.debug("%s No Pitstop required > carFuel:%.2f, remLap:%d\n", car->_name,car->_fuel, car->_remainingLaps);
        m_checkFuel = true;
        return;
    }

    // We don't have enough fuel to end the race need at least one stop.
    if (!m_checkFuel && requiredfuel > 0.0f && car->_fuel < fuelPerLap * 3.0)
    {
        LogSHADOW.debug("%s Pitstop needed to refuel >> reqFuel: %.2f, carFuel: %.2f, remLap: %d\n",
              car->_name, requiredfuel, car->_fuel, car->_remainingLaps);
        m_checkFuel = true;
    }

    return;
}

bool SimpleStrategy::needPitstop(tCarElt* car, tSituation *s)
{
    int m_maxDamage = PIT_DAMMAGE;
    float attvalue = 0.0f;
    // load defined value in xml file of Max Dammage before pitstops for this track
    attvalue = GfParmGetNum(car->_carHandle, SECT_PRIV, PRV_DAMAGE, (char*) NULL, (tdble) PIT_DAMMAGE);
    m_maxDamage = (int)attvalue;
    // Estimated average fuel per lap
    //m_Fuel = GfParmGetNum(car->_carHandle, BT_SECT_PRIV, BT_ATT_FUELPERLAP, (char*) NULL, m_expectedfuelperlap);

    double minFuelFactor;
    minFuelFactor = 1.8;
    if (shortTrack)
    {
        minFuelFactor = 2.15;
    }
    if (qualifRace)
    {
        minFuelFactor = 0.99;
    }

    /* Question makes only sense if there is a pit. */
    if(car->_pit != NULL)
    {
        /* Ideally we shouldn't pit on the last lap...
          just get to the finish line somehow. */
        int lapsToEnd = car->_remainingLaps - car->_lapsBehindLeader;
        if(lapsToEnd > 0)
        {
            // Do we need to refuel?
            double cmpfuel = (fuelPerLap == 0.0f) ? m_expectedfuelperlap : fuelPerLap;
            double reqfuel = lapsToEnd * cmpfuel;

            if(car->_fuel < fuelPerLap * minFuelFactor && car->_fuel < lapsToEnd * fuelPerLap)
            {
                if (!m_checkFuel)
                {
                    LogSHADOW.debug("%s Go to Pit the next lap to refuel: reqFuel=%.2f, carFuel=%.2f, remLap=%d\n",
                          car->_name, reqfuel, car->_fuel, car->_remainingLaps);
                    m_checkFuel = true;
                }
                return true;
            }

            // Do we need to repair and is the pit free?
            needRepair = false;
            if (car->_dammage > m_maxDamage && isPitFree(car))
            {
                needRepair = true;
                if(laps_to_go(car) > 5)
                {
                    if (!m_checkDamage)
                    {
                        LogSHADOW.debug("%s >> Max_damage: %d Car_damage: %d Laps_toGo: %d\n",
                              car->_name, m_maxDamage, car->_dammage, laps_to_go(car));
                        m_checkDamage = true;
                    }
                    return true;
                } else if(laps_to_go(car) <= 5)
                {
                    if(car->_dammage > MAX_DAMAGE)
                    {
                        quickPitstop = true;
                        return true;
                    } else
                    {
                        if (!m_checkDamage)
                        {
                            LogSHADOW.debug("%s Dont Stop At Pit!> Laps_toGo:%d  Car_damage: %d Max_damage: %d\n",
                                  car->_name, laps_to_go(car), car->_dammage, m_maxDamage);
                            m_checkDamage = true;
                            needRepair = false;
                        }
                    }
                }

            }
        }
    }

    return false;
}


bool SimpleStrategy::isPitFree(tCarElt* car)
{
    if (car->_pit != NULL)
    {
        if (car->_pit->pitCarIndex == TR_PIT_STATE_FREE)
        {
            return true;
        }
    }

    return false;
}

int SimpleStrategy::pitRepair(tCarElt* car, tSituation *s)
{
    m_checkDamage = false;
    int damRepair = 0;
    if (needRepair)
    {
        damRepair = car->_dammage;
        if(quickPitstop)
        {
            damRepair = (int)(0.3 * car->_dammage);
        }

        needRepair = false;
    }

    LogSHADOW.debug("# %s repairing %d dammage\n", car->_name, damRepair);
    return damRepair;
}


float SimpleStrategy::pitRefuel(tCarElt* car, tSituation *s)
{
    float fuel;
    float fuelToEnd;
    int lapsToEnd;

    lapsToEnd = car->_remainingLaps - car->_lapsBehindLeader;
    // int inLap = s->_totLaps - car->_remainingLaps;               // Removed 6th April 2015 - Not Used
    fuelToEnd = (tdble)(MIN(getRefuel1(lapsToEnd), getRefuel2(lapsToEnd)));

    //m_remainingstops = int(floor(fuelToEnd / car->_tank));
    m_remainingstops = (int)(fabs(fuelToEnd / car->_tank));
    int num_remStops = m_remainingstops + 1;
    m_fuelperstint = car->_tank - car->_fuel;
    fuel = (tdble)(m_fuelperstint * 0.90);
    double addFuel = fuelPerLap;
    double addMinFuel = fuelPerLap * 0.80;

    if (shortTrack && fuelPerLap < 3.50)
    {
        addFuel = fuelPerLap * 2.0;
        addMinFuel = fuelPerLap;
    }

    countPitStop++;

    if ( fuel_Strat == 2 )
    {
        /* Long track strategy eg Spring*/
        if (strategy_verbose)
        {
            LogSHADOW.debug(">> [PitStrat 2] ");
        }
        LogSHADOW.debug(" %s in Pit to refuel < PitStop %d >\n", car->_name, countPitStop);

        m_fuelperstint = (tdble)((fuelToEnd / num_remStops) + addFuel);

        if (m_remainingstops && (m_fuelperstint / car->_tank > 0.98))
        {
            m_fuelperstint = car->_tank;
        }

        fuel = (tdble)(MAX(MIN(m_fuelperstint - car->_fuel, car->_tank - car->_fuel), 0.0));

        if (!m_remainingstops)
        {
            m_fuelperstint = (tdble)((lapsToEnd * fuelPerLap) + addMinFuel);
        }

    } else
    {
        /* Strategy for short track and track length less than 5000m*/
        if (strategy_verbose)
        {
            LogSHADOW.debug(">> [PitStrat 1] ");
        }

        if (m_remainingstops >= 1)
        {
            m_fuelperstint = (tdble)((fuelToEnd / num_remStops) + addFuel);
        } else if (m_remainingstops <= 0)
        {
            m_fuelperstint = (tdble)((lapsToEnd * fuelPerLap) + addMinFuel);
        }

        fuel = MIN(m_fuelperstint - car->_fuel, car->_tank - car->_fuel);
        LogSHADOW.debug(" %s in Pit to refuel %.2fl < PitStop %d >\n", car->_name, fuel, countPitStop);
    }


    if (fuelToEnd <= car->_fuel)
    {
        m_remainingstops = 0;
        m_fuelperstint = 0.0;
        fuel = 0.0;
    }

#ifdef STRAT_DEBUG
    double totalFuel = s->_totLaps * fuelPerLap;
    double fuelPerPitStop = totalFuel / 4;
    LogSHADOW.debug("[%s()] totalFuel=%.2f fuelPerPitStop=%.2f inLap=%d lapsToEnd=%d m_fuelperstint=%.2f car_fuel=%.2f fuel=%.2f\n",
            __FUNCTION__, totalFuel, fuelPerPitStop, inLap, lapsToEnd, m_fuelperstint, car->_fuel, fuel);
#endif

    if (strategy_verbose) {
        if (!m_remainingstops) {
            LogSHADOW.debug("# Last Pitstop for %s: LapsToEnd: %d, LapsBehindLeader: %d, fuelToEnd: %.2f, FuelperLap: %.2f\n",
                    car->_name, lapsToEnd, car->_lapsBehindLeader, fuelToEnd, fuelPerLap);
        } else {
            LogSHADOW.debug("# [%s()] LapsToEnd: %d, LapsBehindLeader: %d, FuelToEnd: %.2f, FuelperLap: %.2f,\n", __FUNCTION__,
                    lapsToEnd, car->_lapsBehindLeader, fuelToEnd, fuelPerLap);
        }
        LogSHADOW.debug(">> %s ReFueling %.2f,  Fuel needed = %.2f, carFuel= %.2f, remPitStop= %d\n",
                car->_name, fuel, m_fuelperstint, car->_fuel,  m_remainingstops);
        LogSHADOW.debug(" \n");
    }

    lastPitFuel = fuel;
    m_checkFuel = false;
    return fuel;
}

float SimpleStrategy::calcFuel(double totalFuel)
{
    float fuelAtStart;
    float m_lastfuel;
    int nb_pitstop = 0;
    int nb_laps = 0;

#if 0
    if (inRace) {
        /* Now we are in race*/
        if ( totalFuel / maxFuel > numPitstop ) {
            numPitstop += numPitstop + 1;
        }
        double pit_ReFuel = (totalFuel / numPitstop) + fuelPerLap;
        if ( pit_ReFuel > maxFuel ){
            return maxFuel;
        }

        return pit_ReFuel;
    }
#endif

    nb_pitstop = (int)(1 + fabs(totalFuel / maxFuel));
    m_lastfuel = (tdble)(totalFuel / nb_pitstop);  //Max refuel per pit stop
    nb_laps = (int)(1 + floor( m_lastfuel / fuelPerLap));
    fuelAtStart = nb_laps * fuelPerLap;

    return fuelAtStart;
}

double SimpleStrategy::getRefuel1(int laps)
{
    double refuelforrace = laps * fuelPerLap;

    return refuelforrace;
}

double SimpleStrategy::getRefuel2(int laps)
{
    double refuelforrace = laps * avgFuelPerLap;

    return refuelforrace;
}



