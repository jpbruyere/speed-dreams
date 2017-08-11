/***************************************************************************

    file                 : wheel.h
    created              : Sun Mar 19 00:09:18 CET 2000
    copyright            : (C) 2000 by Eric Espie
    email                : torcs@free.fr
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

#ifndef _WHEEL_H__
#define _WHEEL_H__

#include "differential.h"

#undef USE_THICKNESS
#define N_THICKNESS_SEGMENTS 16
#define SEGMENT_RANGE 4
#define PRM_DYNAMIC_CAMBER "steering dynamic camber rate"

typedef struct
{

    /* internal data */
    tSuspension  susp;		/* associated suspension */
    tBrake       brake;		/* associated brake disk */

    /* dynamic */
    glm::vec3	forces;		/* forces acting on car, using car's FOR */
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
#define SIM_WH_FREE     16  /* the wheel is non longer on the car */
#define SIM_WH_BURST    32  /* the wheel has exploded */ 
    float	axleFz;		/* force from axle (anti-roll bar) */
    tTrkLocPos	trkPos;		/* current track position */
    tPosd	relPos;		/* relative pos / GC */
    float	sa;		/* slip angle */
    float	sx;		/* longitudinal slip value */
    float	steer;

    /* static */
    tPosd	staticPos;	/* pos relative to the GC (z is suspension
						   travel at rest) and angles are camber (ax),
						   caster (ay) and toe (az) */
    float	rollCenter;

    float  	weight0;	/* initial weight on this wheel */
    float	tireSpringRate;
    float  	radius;
    float   width;
    float  	mu;
    float  	I;       	/* I = inertial moment of the wheel */
    float  	curI;       	/* Current inertia for the wheel (including transmission) */
    float	mfC;		/* Magic Formula C coeff */
    float	mfB;		/* Magic Formula B coeff */
    float	mfE;		/* Magic Formula E coeff */
    float   mfT;            /* Temperature-dependent coeff */

	/* Tyre wear */
    float   Ca;         /* Adherence coefficient */
    float   T_current;      /* Temperature */
    float   T_operating;    /* Operating temperature */
    float   condition;      /* Tyre condition */
    float   T_range;
	/* Modelling uneven thickness due to wear */ 
#ifdef USE_THICKNESS
	float   thickness[N_THICKNESS_SEGMENTS];
	float   segtemp[N_THICKNESS_SEGMENTS];
#endif

    float	lfMax;		/* Load factor */
    float	lfMin;		/* Load factor */
    float	lfK;		/* Load factor */
    float	opLoad;		/* Operating load */
    float	mass;		/* total wheel mass (incl. brake) (unsprung mass) */
    float	camber;		/* camber, negative toward exterior on both sides */
    float	pressure;	/* tire pressure */
    float   rel_vel;    /* relative velocity - used for realstic suspension movement*/
    float   dynamic_camber; /* steering dynamic camber angle */
    float   bump_force;  /* bumps due to realistic suspension movement */

	/* axis damage */
	float rotational_damage_x;
	float rotational_damage_z;
	float bent_damage_x;
	float bent_damage_z;
    tDynAxis	in;
    tDynAxis	feedBack;

    float	preFn, preFt;
	float   Em; // estimate of mass
	float   s_old;
	float   F_old;

    glm::vec3   	normal;	   	

} tWheel;

    

#endif /* _WHEEL_H__ */ 



