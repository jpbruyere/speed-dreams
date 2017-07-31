/***************************************************************************

    file                 : hymie.cpp
    created              : Wed Jan 8 18:31:16 CET 2003
    copyright            : (C) 2007 Andrew Sumner, 2002-2004 Bernhard Wymann
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

#ifdef _WIN32
#include <windows.h>
#endif

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
#include "name.h"

#define BUFSIZE 20
#define NBBOTS 10

#ifdef ROB_SECT_ARBITRARY
static char const* botname[11] = {
 "SC: Arne Fischer",
 "SC: Hans Meyer",
 "SC: Jackie Graham",
 "SC: Steve Magson",
 "SC: Tony Davies",
 "SC: Ken Rayner",
 "SC: Mark Duncan",
 "SC: Chuck Davis Jr",
 "SC: Stefan Larsson",
 "SC: Don Nelson",
 "hymie"};
#else
static char const* botname[NBBOTS] = {
 "SC: Arne Fischer",
 "SC: Hans Meyer",
 "SC: Jackie Graham",
 "SC: Steve Magson",
 "SC: Tony Davies",
 "SC: Ken Rayner",
 "SC: Mark Duncan",
 "SC: Chuck Davis Jr",
 "SC: Stefan Larsson",
 "SC: Don Nelson" };
#endif

#ifdef ROB_SECT_ARBITRARY
static Driver **driver;
static int driverAlloc = 0;
#else
static Driver *driver[NBBOTS];
#endif

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s);
static void newRace(int index, tCarElt* car, tSituation *s);
static void drive(int index, tCarElt* car, tSituation *s);
static int pitcmd(int index, tCarElt* car, tSituation *s);
static void shutdown(int index);
static int InitFuncPt(int index, void *pt);
static void endRace(int index, tCarElt *car, tSituation *s);


// Module entry point.
extern "C" int hymie(tModInfo *modInfo)
{
	int i;

	// Clear all structures.
#ifdef ROB_SECT_ARBITRARY
	memset(modInfo, 0, 11*sizeof(tModInfo));

	for (i = 0; i < 11; i++) {
		if (i < NBBOTS || i == 10) {
			modInfo[i].name    = botname[i];  			// name of the module (short).
			modInfo[i].desc    = botname[i];			// Description of the module (can be long).
			modInfo[i].fctInit = InitFuncPt;			// Init function.
			modInfo[i].gfId    = ROB_IDENT;				// Supported framework version.
			modInfo[i].index   = i+1;						// Indices from 0 to 9.
		}
	}
#else
	memset(modInfo, 0, NBBOTS*sizeof(tModInfo));

	for (i = 0; i < NBBOTS; i++) {
		modInfo[i].name    = botname[i];  			// name of the module (short).
		modInfo[i].desc    = botname[i];			// Description of the module (can be long).
		modInfo[i].fctInit = InitFuncPt;			// Init function.
		modInfo[i].gfId    = ROB_IDENT;				// Supported framework version.
		modInfo[i].index   = i+1;						// Indices from 0 to 9.
	}
#endif
	return 0;
}


// Module interface initialization.
static int InitFuncPt(int index, void *pt)
{
	tRobotItf *itf = (tRobotItf *)pt;
#ifdef ROB_SECT_ARBITRARY
	int xx;
	Driver **copy;

	//Make sure enough data is allocated
	if (driverAlloc < index) {
		copy = new Driver*[index];
		for (xx = 0; xx < driverAlloc; ++xx)
			copy[xx] = driver[xx];
		for (xx = driverAlloc; xx < index; ++xx)
			copy[xx] = NULL;
		if (driverAlloc > 0)
			delete []driver;
		driver = copy;
		driverAlloc = index;
	}
#endif

	// Create robot instance for index.
	driver[index-1] = new Driver(index);
	itf->rbNewTrack = initTrack;	// Give the robot the track view called.
	itf->rbNewRace  = newRace;		// Start a new race.
	itf->rbDrive    = drive;		// Drive during race.
	itf->rbPitCmd   = pitcmd;		// Pit commands.
	itf->rbEndRace  = endRace;		// End of the current race.
	itf->rbShutdown = shutdown;		// Called before the module is unloaded.
	itf->index      = index;		// Index used if multiple interfaces.
	return 0;
}


// Called for every track change or new race.
static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s)
{
	driver[index-1]->initTrack(track, carHandle, carParmHandle, s);
}


// Start a new race.
static void newRace(int index, tCarElt* car, tSituation *s)
{
	driver[index-1]->newRace(car, s);
}


// Drive during race.
static void drive(int index, tCarElt* car, tSituation *s)
{
	driver[index-1]->drive(s);
}


// Pitstop callback.
static int pitcmd(int index, tCarElt* car, tSituation *s)
{
	return driver[index-1]->pitCommand(s);
}


// End of the current race.
static void endRace(int index, tCarElt *car, tSituation *s)
{
	driver[index-1]->endRace(s);
}


// Called before the module is unloaded.
static void shutdown(int index)
{
#ifdef ROB_SECT_ARBITRARY
	int xx, yy;
	Driver **copy;
#endif

	delete driver[index-1];
#ifdef ROB_SECT_ARBITRARY
	driver[index-1] = NULL;
	
	for (yy = driverAlloc - 1; yy >= 0; --yy) {
		if (driver[yy]) {
			if (yy == driverAlloc - 1)
				break; //Nothing to do: last item of array isn't empty
			copy = new Driver*[yy+1];
			for (xx = 0; xx <= yy; ++xx)
				copy[xx] = driver[xx];
			delete []driver;
			driver = copy;
			driverAlloc = yy + 1;
			break;
		} else if (yy == 0) {
			delete []driver;
			driverAlloc = 0;
			break;
		}
	}
#endif
}

