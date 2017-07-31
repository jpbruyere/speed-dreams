/***************************************************************************

    file        : mouse_2006.cpp
    created     : 9 Apr 2006
    copyright   : (C) 2006 Tim Foden

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
#include <portability.h>

#include "Vec3d.h"
#include "Driver.h"
#include "Shared.h"

static Shared   s_shared;

// Traditional TORCS Interface
static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s);
static void newRace(int index, tCarElt* car, tSituation *s);
static void drive(int index, tCarElt* car, tSituation *s);
static int  pitcmd(int index, tCarElt* car, tSituation *s);
static void shutdown(int index);
static void endRace(int index, tCarElt *car, tSituation *s);
static int  InitFuncPt(int index, void *pt);
extern "C" int shadow(tModInfo *modInfo);

// Speed Dreams Interface
static char const* defaultBotName[MAXNBBOTS] =
{
    "driver 1",  "driver 2",  "driver 3",  "driver 4",  "driver 5",
    "driver 6",  "driver 7",  "driver 8",  "driver 9",  "driver 10",
    "driver 11", "driver 12", "driver 13", "driver 14", "driver 15",
    "driver 16", "driver 17", "driver 18", "driver 19", "driver 20"
};

static char const* defaultBotDesc[MAXNBBOTS] =
{
    "driver 1",  "driver 2",  "driver 3",  "driver 4",  "driver 5",
    "driver 6",  "driver 7",  "driver 8",  "driver 9",  "driver 10",
    "driver 11", "driver 12", "driver 13", "driver 14", "driver 15",
    "driver 16", "driver 17", "driver 18", "driver 19", "driver 20"
};

// Max length of a drivername
static const int DRIVERLEN = 32;
// Buffer for driver's names defined in robot's xml-file
static char DriverNames[DRIVERLEN * MAXNBBOTS];
// Buffer for driver's descriptions defined in robot's xml-file
static char DriverDescs[DRIVERLEN * MAXNBBOTS];
// Buffer for car name defined in robot's xml-file
static char CarNames[DRIVERLEN * MAXNBBOTS];
// Array of drivers
static TDriver *driver[MAXNBBOTS];

// Number of drivers defined in robot's xml-file
static int NBBOTS = 0;                           // Still unknown
// Robot's name
static char nameBuffer[BUFSIZE];                 // Buffer for robot's name
static const char* robot_name = nameBuffer;       // Pointer to robot's name
// Robot's xml-filename
static char pathBuffer[BUFSIZE];                 // Buffer for xml-filename
static const char* xml_path = pathBuffer;        // Pointer to xml-filename

// Save start index offset from robot's xml file
static int indexOffset = 0;
// Marker for undefined drivers to be able to comment out drivers
// in the robot's xml-file between others, not only at the end of the list
char undefined[] = "undefined";

static int robot_type;  //Decide if TRB, SC, GP36, LS or some other driver

// The "SHADOW" logger instance
GfLogger* PLogSHADOW = 0;

////////////////////////////////////////////////////////////
// Utility functions
////////////////////////////////////////////////////////////

// Set robots's name
static void setRobotName(const char *name)
{
    strncpy(nameBuffer, name, BUFSIZE);
    LogSHADOW.info("Robot Name: >%s<\n", robot_name);
}


// name: getFileHandle
// Obtains the file handle for the robot XML file,
//  trying the installation path first, then
//  the global one, if the previous attempt failed.
// @param
// @return file handler for the robot XML file
void* getFileHandle()
{
    // First we try to use the directories relative to the installation path
    snprintf(pathBuffer, BUFSIZE, "%sdrivers/%s/%s.xml", GetLocalDir(), robot_name, robot_name);

    // Test local installation path
    void *robot_settings = GfParmReadFile(xml_path, GFPARM_RMODE_STD);

    if (!robot_settings)
    {
        // If not found, use global installation path
        snprintf(pathBuffer, BUFSIZE, "%sdrivers/%s/%s.xml", GetDataDir(), robot_name, robot_name);
        robot_settings = GfParmReadFile(xml_path, GFPARM_RMODE_STD);
    }

    return robot_settings;
}

////////////////////////////////////////////////////////////
// Carset specific init functions
////////////////////////////////////////////////////////////

// Schismatic init for shadow_trb1
void SetupSHADOW_trb1()
{
    // Add shadow_trb1 specific initialization here
    robot_type = SHADOW_TRB1;
};

// Schismatic init for shadow_ls2
void SetupSHADOW_ls2()
{
    // Add shadow_ls2 specific initialization here
    robot_type = SHADOW_LS2;
};


// Schismatic init for shadow_sc
void SetupSHADOW_sc()
{
    // Add shadow_sc specific initialization here
    robot_type = SHADOW_SC;
};

// Schismatic init for shadow_sc
void SetupSHADOW_srw()
{
    // Add shadow_sc specific initialization here
    robot_type = SHADOW_SRW;
};


// Schismatic init for shadow_ls1
void SetupSHADOW_ls1()
{
    // Add shadow_ls1 specific initialization here
    robot_type = SHADOW_LS1;
};


// Schismatic init for shadow_36GP
void SetupSHADOW_36GP()
{
    // Add shadow_36GP specific initialization here
    robot_type = SHADOW_36GP;
};

// Schismatic init for shadow_36GP
void SetupSHADOW_67GP()
{
    // Add shadow_36GP specific initialization here
    robot_type = SHADOW_67GP;
};

// Schismatic init for shadow_rs
void SetupSHADOW_rs()
{
    // Add usr_RS specific initialization here
    robot_type = SHADOW_RS;
};

// Schismatic init for shadow_lp1
void SetupSHADOW_lp1()
{
    // Add shadow_LP1 specific initialization here
    robot_type = SHADOW_LP1;
};

// Schismatic init for shadow_mpa1
void SetupSHADOW_mpa1()
{
    // Add shadow_mpa1 specific initialization here
    robot_type = SHADOW_MPA1;
};

// Schismatic init for shadow_mpa11
void SetupSHADOW_mpa11()
{
    // Add shadow_mpa11 specific initialization here
    robot_type = SHADOW_MPA11;
};

// Schismatic init for shadow_mpa12
void SetupSHADOW_mpa12()
{
    // Add shadow_mpa1 specific initialization here
    robot_type = SHADOW_MPA12;
};


////////////////////////////////////////////////////////////
// Carset specific entry points (functions)
////////////////////////////////////////////////////////////

// Schismatic entry point for shadow_trb1
extern "C" int shadow_trb1(tModInfo *ModInfo)
{
    int ret = -1;
    setRobotName("shadow_trb1");
    robot_type = SHADOW_TRB1;
    void *robot_settings = getFileHandle();
    if (robot_settings)
    {
        ret = shadow(ModInfo);
    }

    return ret;
}


// Schismatic entry point for shadow_sc
extern "C" int shadow_sc(tModInfo *ModInfo)
{
    int ret = -1;
    setRobotName("shadow_sc");
    robot_type = SHADOW_SC;
    void *robot_settings = getFileHandle();
    if (robot_settings)
    {
        ret = shadow(ModInfo);
    }

    return ret;
}

// Schismatic entry point for shadow_srw
extern "C" int shadow_srw(tModInfo *ModInfo)
{
    int ret = -1;
    setRobotName("shadow_srw");
    robot_type = SHADOW_SRW;
    void *robot_settings = getFileHandle();
    if (robot_settings)
    {
        ret = shadow(ModInfo);
    }

    return ret;
}

// Schismatic entry point for shadow_ls2
extern "C" int shadow_ls2(tModInfo *ModInfo)
{
    int ret = -1;
    setRobotName("shadow_ls2");
    robot_type = SHADOW_LS2;
    void *robot_settings = getFileHandle();
    if (robot_settings)
    {
        ret = shadow(ModInfo);
    }

    return ret;
}

// Schismatic entry point for shadow_ls1
extern "C" int shadow_ls1(tModInfo *ModInfo)
{
    int ret = -1;
    setRobotName("shadow_ls1");
    robot_type = SHADOW_LS1;
    void *robot_settings = getFileHandle();
    if (robot_settings)
    {
        ret = shadow(ModInfo);
    }

    return ret;
}

// Schismatic entry point for shadow_mpa1
extern "C" int shadow_mpa1(tModInfo *ModInfo)
{
    int ret = -1;
    setRobotName("shadow_mpa1");
    robot_type = SHADOW_MPA1;
    void *robot_settings = getFileHandle();
    if (robot_settings)
    {
        ret = shadow(ModInfo);
    }

    return ret;
}

// Schismatic entry point for shadow_mpa11
extern "C" int shadow_mpa11(tModInfo *ModInfo)
{
    int ret = -1;
    setRobotName("shadow_mpa11");
    robot_type = SHADOW_MPA11;
    void *robot_settings = getFileHandle();
    if (robot_settings)
    {
        ret = shadow(ModInfo);
    }

    return ret;
}

// Schismatic entry point for shadow_mpa12
extern "C" int shadow_mpa12(tModInfo *ModInfo)
{
    int ret = -1;
    setRobotName("shadow_mpa12");
    robot_type = SHADOW_MPA12;
    void *robot_settings = getFileHandle();
    if (robot_settings)
    {
        ret = shadow(ModInfo);
    }

    return ret;
}

// Schismatic entry point for shadow_36GP
extern "C" int shadow_36GP(tModInfo *ModInfo)
{
    int ret = -1;
    setRobotName("shadow_36GP");
    robot_type = SHADOW_36GP;
    void *robot_settings = getFileHandle();
    if (robot_settings)
    {
        ret = shadow(ModInfo);
    }

    return ret;
}

// Schismatic entry point for shadow_36GP
extern "C" int shadow_67GP(tModInfo *ModInfo)
{
    int ret = -1;
    setRobotName("shadow_67GP");
    robot_type = SHADOW_67GP;
    void *robot_settings = getFileHandle();
    if (robot_settings)
    {
        ret = shadow(ModInfo);
    }

    return ret;
}

// Schismatic entry point for shadow_rs
extern "C" int shadow_rs(tModInfo *ModInfo)
{
    int ret = -1;
    setRobotName("shadow_rs");
    robot_type = SHADOW_RS;
    void *robot_settings = getFileHandle();
    if (robot_settings)
    {
        ret = shadow(ModInfo);
    }

    return ret;
}

// Schismatic entry point for shadow_LP1
extern "C" int shadow_lp1(tModInfo *ModInfo)
{
    int ret = -1;
    setRobotName("shadow_lp1");
    robot_type = SHADOW_LP1;
    void *robot_settings = getFileHandle();
    if (robot_settings)
    {
        ret = shadow(ModInfo);
    }

    return ret;
}

////////////////////////////////////////////////////////////
// General entry point (new scheme)
////////////////////////////////////////////////////////////
// Module entry point (new fixed name scheme), step #1.
// Extended for use with schismatic robots
extern "C" int moduleWelcome(const tModWelcomeIn* welcomeIn,
                             tModWelcomeOut* welcomeOut)
{
    PLogSHADOW = GfLogger::instance("SHADOW");
    LogSHADOW.debug("\n#Interface Version: %d.%d\n",
                     welcomeIn->itfVerMajor,welcomeIn->itfVerMinor);
    // Save module name and loadDir, and determine module XML file pathname.
    setRobotName(welcomeIn->name);

    // Filehandle for robot's xml-file
    void *robot_settings = getFileHandle();
    LogSHADOW.debug("Robot XML-Path: %s\n\n", xml_path);

    // Let's look what we have to provide here
    if (robot_settings)
    {
        char section_buf[BUFSIZE];
        const char *section = section_buf;
        snprintf(section_buf, BUFSIZE, "%s/%s/%d", ROB_SECT_ROBOTS, ROB_LIST_INDEX, 0);

        // Try to get first driver from index 0
        const char *driver_name = GfParmGetStrNC(robot_settings, section, ROB_ATTR_NAME, undefined);

        // Check whether index 0 is used as start index
        if (strncmp(driver_name, undefined, strlen(undefined)) != 0)
        {
            // Teams xml file uses index 0, 1, ..., N - 1
            indexOffset = 0;
        }
        else
        {
            // Teams xml file uses index 1, 2, ..., N
            indexOffset = 1;
        }

        // Loop over all possible drivers, clear all buffers,
        // save defined driver names and desc.
        for (int i = 0; i < MAXNBBOTS; ++i)
        {
            // Clear buffers
            memset(&DriverNames[i * DRIVERLEN], 0, DRIVERLEN);
            memset(&DriverDescs[i * DRIVERLEN], 0, DRIVERLEN);
            memset(&CarNames[i * DRIVERLEN], 0, DRIVERLEN);

            snprintf(section_buf, BUFSIZE, "%s/%s/%d", ROB_SECT_ROBOTS, ROB_LIST_INDEX, i + indexOffset);
            const char *driver_name = GfParmGetStr(robot_settings, section, ROB_ATTR_NAME, undefined);

            if (strncmp(driver_name, undefined, strlen(undefined)) != 0)
            {
                // This driver is defined in robot's xml-file
                strncpy(&DriverNames[i * DRIVERLEN], driver_name, DRIVERLEN - 1);
                const char *driver_desc = GfParmGetStr(robot_settings, section, ROB_ATTR_DESC, defaultBotDesc[i]);
                strncpy(&DriverDescs[i * DRIVERLEN], driver_desc, DRIVERLEN - 1);

                const char *car_name = GfParmGetStr(robot_settings, section, ROB_ATTR_CAR, "nocar");
                strncpy(&CarNames[i * DRIVERLEN], car_name, DRIVERLEN - 1);

                NBBOTS = i + 1;
            }
        }
    }
    else
    {
        // For schismatic robots NBBOTS is unknown!
        // Handle error here
        NBBOTS = 0;
        // But this is not considered a real failure of moduleWelcome !
    }
    LogSHADOW.debug("NBBOTS: %d (of %d)\n", NBBOTS, MAXNBBOTS);

	if (strncmp(robot_name,"shadow_sc", strlen("shadow_sc")) == 0)
        SetupSHADOW_sc();
    else if (strncmp(robot_name,"shadow_srw", strlen("shadow_srw")) == 0)
        SetupSHADOW_srw();
    else if (strncmp(robot_name,"shadow_ls1", strlen("shadow_ls1")) == 0)
        SetupSHADOW_ls1();
    else if (strncmp(robot_name,"shadow_ls2", strlen("shadow_ls2")) == 0)
        SetupSHADOW_ls2();
    else if (strncmp(robot_name,"shadow_36GP", strlen("shadow_36GP")) == 0)
        SetupSHADOW_36GP();
    else if (strncmp(robot_name,"shadow_67GP", strlen("shadow_67GP")) == 0)
        SetupSHADOW_67GP();
    else if (strncmp(robot_name,"shadow_rs", strlen("shadow_rs")) == 0)
        SetupSHADOW_rs();
    else if (strncmp(robot_name,"shadow_lp1", strlen("shadow_lp1")) == 0)
        SetupSHADOW_lp1();
    else if (strncmp(robot_name,"shadow_mpa11", strlen("shadow_mpa11")) == 0)
        SetupSHADOW_mpa11();
	else if (strncmp(robot_name,"shadow_mpa12", strlen("shadow_mpa12")) == 0)
        SetupSHADOW_mpa12();
	else if (strncmp(robot_name, "shadow_mpa1", strlen("shadow_mpa1")) == 0)
        SetupSHADOW_mpa1();
	else 
        SetupSHADOW_trb1();


    // Set max nb of interfaces to return.
    welcomeOut->maxNbItf = NBBOTS;

    return 0;
}


// Module entry point (new fixed name scheme).
extern "C" int moduleInitialize(tModInfo *modInfo)
{
    LogSHADOW.debug("\n\nshadow::moduleInitialize, from %s ...\n", xml_path);
    LogSHADOW.debug("NBBOTS: %d (of %d)\n", NBBOTS, MAXNBBOTS);

    // Clear all structures.
    memset(modInfo, 0, NBBOTS*sizeof(tModInfo));
    int i;

    for (i = 0; i < NBBOTS; ++i)
    {
        modInfo[i].name = &DriverNames[i * DRIVERLEN];
        modInfo[i].desc = &DriverDescs[i * DRIVERLEN];
        modInfo[i].fctInit = InitFuncPt;   // Init function.
        modInfo[i].gfId    = ROB_IDENT;    // Supported framework version.
        modInfo[i].index   = i + indexOffset;  // Indices depend on xml-file
    }

    LogSHADOW.debug("... Initialized %d from %s\n\n\n", i, xml_path);

    return 0;
}


// Module exit point (new fixed name scheme).
extern "C" int moduleTerminate()
{
    LogSHADOW.debug("Terminated shadow\n");
    return 0;
}


// Module interface initialization.
static int InitFuncPt(int index, void *pt)
{
    tRobotItf *itf = reinterpret_cast<tRobotItf*>(pt);

    // Create robot instance for index.
    driver[index-indexOffset] = new TDriver(index, robot_type);
    driver[index-indexOffset]->SetBotName(&CarNames[(index-indexOffset)*DRIVERLEN]);

    itf->rbNewTrack = initTrack;  // Give the robot the track view called.
    itf->rbNewRace  = newRace;    // Start a new race.
    itf->rbDrive    = drive;      // Drive during race.
    itf->rbPitCmd   = pitcmd;     // Pit commands.
    itf->rbEndRace  = endRace;    // End of the current race.
    itf->rbShutdown = shutdown;   // Called before the module is unloaded.
    itf->index      = index;      // Index used if multiple interfaces.

    return 0;
}


// Called for every track change or new race.
static void initTrack(int index, tTrack* track, void *carHandle,
                      void **carParmHandle, tSituation *s)
{
    driver[index-indexOffset]->SetShared( &s_shared );
    driver[index-indexOffset]->InitTrack(track, carHandle, carParmHandle, s);
}


// Start a new race.
static void newRace(int index, tCarElt* car, tSituation *s)
{
    driver[index-indexOffset]->NewRace(car, s);
}


// Drive during race.
static void drive(int index, tCarElt* car, tSituation *s)
{
    driver[index-indexOffset]->Drive(s);
}


// Pitstop callback.
static int pitcmd(int index, tCarElt* car, tSituation *s)
{
    return driver[index-indexOffset]->PitCmd(s);
}


// End of the current race.
static void endRace(int index, tCarElt *car, tSituation *s)
{
    driver[index-indexOffset]->EndRace(s);
}


// Called before the module is unloaded.
static void shutdown(int index)
{
    driver[index-indexOffset]->Shutdown();
    delete driver[index-indexOffset];
}


////////////////////////////////////////////////////////////
// Ye Olde Interface
////////////////////////////////////////////////////////////

// Module entry point (Torcs backward compatibility scheme).
extern "C" int shadow(tModInfo *modInfo)
{
    NBBOTS = 10;
    memset(DriverNames, 0, NBBOTS * DRIVERLEN);
    memset(DriverDescs, 0, NBBOTS * DRIVERLEN);

    // Filehandle for robot's xml-file
    void *robot_settings = getFileHandle();

    // Let's look what we have to provide here
    if (robot_settings)
    {
        char SectionBuf[BUFSIZE];
        char *Section = SectionBuf;

        snprintf(SectionBuf, BUFSIZE, "%s/%s/%d", ROB_SECT_ROBOTS, ROB_LIST_INDEX, 0);

        for (int i = 0; i < NBBOTS; ++i)
        {
            const char *DriverName = GfParmGetStr(robot_settings, Section, ROB_ATTR_NAME, defaultBotName[i]);
            strncpy(&DriverNames[i * DRIVERLEN], DriverName, DRIVERLEN - 1);

            const char *DriverDesc = GfParmGetStr(robot_settings, Section, ROB_ATTR_DESC, defaultBotDesc[i]);
            strncpy(&DriverDescs[i * DRIVERLEN], DriverDesc, DRIVERLEN - 1);
        }
    }

    return moduleInitialize(modInfo);
}


// Module exit point (Torcs backward compatibility scheme).
extern "C" int shadowShut()
{
    return moduleTerminate();
}
