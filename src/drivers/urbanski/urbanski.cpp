/***************************************************************************

	file                 : urbanski.cpp
	created              : Mon Nov 3 01:13:44 CET 2014
	copyright            : (C) 2014 Patryk Urbanski

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

static const int DRIVERS = 2;
static Driver driver[DRIVERS];

static void initTrack(int index, tTrack *track, void *carHandle, void **carParmHandle, tSituation *s);
static void newrace(int index, tCarElt *car, tSituation *s);
static void drive(int index, tCarElt *car, tSituation *s);
static void endrace(int index, tCarElt *car, tSituation *s);
static void shutdown(int index);
static int  InitFuncPt(int index, void *pt);

extern "C" int moduleInitialize(tModInfo *modInfo) {
	memset(modInfo, 0, DRIVERS*sizeof(tModInfo));

	for (int i = 0; i < DRIVERS; i++, modInfo++) {
		modInfo->name    = "urbanski";		/* name of the module (short) */
		modInfo->desc    = "";	/* description of the module (can be long) */
		modInfo->fctInit = InitFuncPt;		/* init function */
		modInfo->gfId    = ROB_IDENT;		/* supported framework version */
		modInfo->index   = i;
	}

	GfOut("Initialized urbanski\n");
	return 0;
}

extern "C" int moduleWelcome(const tModWelcomeIn* welcomeIn, tModWelcomeOut* welcomeOut) {
	welcomeOut->maxNbItf = DRIVERS;
	return 0;
}

extern "C" int moduleTerminate() {
	GfOut("Terminated urbanski\n");
	return 0;
}

/*
 * Module entry point
 */
extern "C" int urbanski(tModInfo *modInfo) {
	return moduleInitialize(modInfo);
}

/*
 * Module exit point
 */
extern "C" int urbanskiShut() {
	return moduleTerminate();
}

/* Module interface initialization. */
static int InitFuncPt(int index, void *pt) {
	tRobotItf *itf  = (tRobotItf *)pt;

	itf->rbNewTrack = initTrack; /* Give the robot the track view called */
				 /* for every track change or new race */
	itf->rbNewRace  = newrace; 	 /* Start a new race */
	itf->rbDrive    = drive;	 /* Drive during race */
	itf->rbPitCmd   = NULL;
	itf->rbEndRace  = endrace;	 /* End of the current race */
	itf->rbShutdown = shutdown;	 /* Called before the module is unloaded */
	itf->index      = index; 	 /* Index used if multiple interfaces */
	driver[index] = Driver();
	return 0;
}

/* Called for every track change or new race. */
static void initTrack(int index, tTrack *track, void *carHandle, void **carParmHandle, tSituation *s) {
	driver[index].setTrack(track, carParmHandle);
}

/* Start a new race. */
static void newrace(int index, tCarElt *car, tSituation *s) {
	driver[index].setCar(car);
}

/* Drive during race. */
static void drive(int index, tCarElt *car, tSituation *s) {
	driver[index].drive(s);
}

/* End of the current race */
static void endrace(int index, tCarElt *car, tSituation *s) {
	driver[index].endRace();
}

/* Called before the module is unloaded */
static void shutdown(int index) {
}

