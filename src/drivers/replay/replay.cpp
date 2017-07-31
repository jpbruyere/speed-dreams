/***************************************************************************

    file                 : replay.cpp
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

#define BUFSIZE 20
#define NBBOTS 10

static char const* botname[] = {
 "RP: Replay 1",
 "RP: Replay 2",
 "RP: Replay 3",
 "RP: Replay 4",
 "RP: Replay 5",
 "RP: Replay 6",
 "RP: Replay 7",
 "RP: Replay 8",
 "RP: Replay 9",
 "RP: Replay 10"
};

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s);
static void newRace(int index, tCarElt* car, tSituation *s);
static void drive(int index, tCarElt* car, tSituation *s);
static int pitcmd(int index, tCarElt* car, tSituation *s);
static void shutdown(int index);
static int InitFuncPt(int index, void *pt);
static void endRace(int index, tCarElt *car, tSituation *s);


// Module entry point.
extern "C" int replay(tModInfo *modInfo)
{
	int i;

	// Clear all structures.
	memset(modInfo, 0, NBBOTS*sizeof(tModInfo));

	for (i = 0; i < NBBOTS; i++) {
		modInfo[i].name    = botname[i];  			// name of the module (short).
		modInfo[i].desc    = botname[i];			// Description of the module (can be long).
		modInfo[i].fctInit = InitFuncPt;			// Init function.
		modInfo[i].gfId    = ROB_IDENT;				// Supported framework version.
		modInfo[i].index   = i+1;						// Indices from 0 to 9.
	}

	return 0;
}


// Module interface initialization.
static int InitFuncPt(int index, void *pt)
{
	tRobotItf *itf = (tRobotItf *)pt;

	// Create robot instance for index.
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
	//driver[index-1]->initTrack(track, carHandle, carParmHandle, s);
}


// Start a new race.
static void newRace(int index, tCarElt* car, tSituation *s)
{
	//driver[index-1]->newRace(car, s);
}


// Drive during race.
static void drive(int index, tCarElt* car, tSituation *s)
{
	//driver[index-1]->drive(s);
}


// Pitstop callback.
static int pitcmd(int index, tCarElt* car, tSituation *s)
{
	//return driver[index-1]->pitCommand(s);
   return 0;
}


// End of the current race.
static void endRace(int index, tCarElt *car, tSituation *s)
{
	//driver[index-1]->endRace(s);
}


// Called before the module is unloaded.
static void shutdown(int index)
{

}
