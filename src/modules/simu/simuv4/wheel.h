/***************************************************************************

    file                 : wheel.h
    created              : Sun Mar 19 00:09:18 CET 2000
    copyright            : (C) 2000 by Eric Espie
    email                : torcs@free.fr
    version              : $Id: wheel.h 3087 2010-11-03 23:42:34Z kakukri $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef _WHEEL_H__
#define _WHEEL_H__

#include "differential.h"

typedef struct
{

    /* internal data */
    tSuspension  susp;		/* associated suspension */
    tBrake       brake;		/* associated brake disk */

    /* dynamic */
    glm::vec3	forces;		/* forces acting on car */
    glm::vec3	torques;	/* torques acting on car (gyroscopic forces) */
    float   torqueAlign;  /* torque for force feedback from magic formula */
    float	rollRes;	/* Rolling Resistance (summed over the car) */
    float	rideHeight;	/* height of the bottom of the car */
    float	zRoad;		/* z of the road */
    glm::vec3   	pos;	   	/* world related */
    glm::vec3	bodyVel;	/* world related */
    float  	driveTq;   	/* engine torque */
    float	vt;

    float  	spinTq;		/* spin torque feedback */
    float  	spinVel;   	/* spin velocity */
    float  	prespinVel;   	/* spin velocity */
    int     	state;     	/* wheel state */
    /* 1 and 2 are for suspension state */
#define SIM_WH_SPINNING 4	/* the wheel is spinning */
#define SIM_WH_LOCKED   8	/* the wheel is locked */
#define SIM_WH_INAIR   16   /* the wheel is in the air */
    float	axleFz;		/* force from axle (anti-roll bar) */
    float   axleFz3rd;  /* force from axle (3rd/heave spring) */
    tTrkLocPos	trkPos;		/* current track position */
    tPosd	relPos;		/* relative pos / GC */
    float	sa;		/* slip angle */
    float	sx;		/* longitudinal slip value */
    float	steer;
    
    /* static */
    tPosd	staticPos;	/* pos relative to the GC (z is suspension travel at rest) */
				/* and angles are camber (ax), caster (ay) and toe (az) */
    float   cosax, sinax;/*cosinus and sinus of relPos.ax*/

    float  	weight0;	/* initial weight on this wheel */
    float	tireSpringRate;
    float  	radius;
    float  	mu;
    float  	I;       	/* I = inertial moment of the wheel */
    float  	curI;       	/* Current inertia for the wheel (including transmission) */
    float	mfC;		/* Magic Formula C coeff */
    float	mfB;		/* Magic Formula B coeff */
    float	mfE;		/* Magic Formula E coeff */
    float	lfMax;		/* Load factor */
    float	lfMin;		/* Load factor */
    float	lfK;		/* Load factor */
    float	opLoad;		/* Operating load */
    float   AlignTqFactor; /* aligning torque factor */
    float	mass;		/* total wheel mass (incl. brake) (unsprung mass) */
    float	camber;		/* camber, negative toward exterior on both sides */
    float	pressure;	/* tire pressure */
    
    float   Ttire;      /* tire temperature in K */
    float   Topt;       /* optimal temperature in K, where mu maximal */
    float   Tinit;      /* initial tire temperature, right after pit or at start */
    float   muTmult;    /* mu = mumax * (1 - muTmult*(T-Topt)^2) */
    float   heatingm;   /* heating multiplier */
    float   aircoolm;   /* air cooling multiplier */
    float   speedcoolm; /* how aircoolm increases with speed */
    float   wearrate; /* degradation multiplier */
    float   treadDepth; /* tread depth, between 0 and 1 */
    float   critTreadDepth; /* critical tread depth, when grip falls off suddenly */
    float   muTDmult[2]; /* mu is multiplied by muTDmult[i]*treadDepth+muTDoffset[i] */
    float   muTDoffset[2];

    tDynAxis	in;
    tDynAxis	feedBack;

    float	preFn, preFt;
} tWheel;

    

#endif /* _WHEEL_H__ */ 



