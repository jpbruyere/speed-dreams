/***************************************************************************

    file                 : transmission.h
    created              : Mon Apr 16 16:04:36 CEST 2001
    copyright            : (C) 2001 by Eric Espi�
    email                : Eric.Espie@torcs.org
    version              : $Id: transmission.h 2917 2010-10-17 19:03:40Z pouillot $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
 
/** @file    
    		
    @author	<a href=mailto:torcs@free.fr>Eric Espie</a>
    @version	$Id: transmission.h 2917 2010-10-17 19:03:40Z pouillot $
*/

#ifndef _TRANSMISSION_H_
#define _TRANSMISSION_H_

typedef struct 
{
    int		gear;
    int		gearMin;
    int		gearMax;
    int		gearNext;
    float   shiftTime; /* time of shifting in sec */
    float   timeToEngage; /* time to engage gearNext, in sec */
} tGearbox;

typedef struct
{
    int		state;
#define CLUTCH_APPLIED	 1
#define CLUTCH_RELEASED  0
#define CLUTCH_RELEASING 2
    int		mode;
#define CLUTCH_AUTO	0
#define CLUTCH_MANUAL	1
    float	timeToRelease;	/* remaining time before releasing the clutch pedal */
    float	releaseTime;	/* time needed for releasing the clutch pedal */
    float	transferValue;	/* 1.0 -> released, 0.0 -> applied */
} tClutch;

typedef struct
{
    tGearbox	gearbox;
    tClutch	clutch;
    int		type;
#define TRANS_RWD	0
#define TRANS_FWD	1
#define TRANS_4WD	2
    float	overallRatio[MAX_GEARS];	/* including final drive ratio */
    float   gearI[MAX_GEARS];       /* raw gear inertia */
    float	driveI[MAX_GEARS];		/* Inertia (including engine) */
    float	freeI[MAX_GEARS];		/* Inertia when clutch is applied (wheels side) */
    float	gearEff[MAX_GEARS];		/* Gear Efficiency */
    float	curOverallRatio;
    float	curI;

#define TRANS_FRONT_DIFF	0
#define TRANS_REAR_DIFF		1
#define TRANS_CENTRAL_DIFF	2
    tDifferential	differential[3];
} tTransmission;


#endif /* _TRANSMISSION_H_ */ 



