/***************************************************************************

    file                 : engine.h
    created              : Sun Mar 19 00:07:07 CET 2000
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

#ifndef _ENGINE_H_
#define _ENGINE_H_

typedef struct {
    float rads;
    float a;
    float b;
} tEngineCurveElem;

typedef struct {
    float		maxTq;
	float       maxPw;
	float       rpmMaxPw;
	float       TqAtMaxPw;
	float       rpmMaxTq;
    int			nbPts;
    tEngineCurveElem	*data;
} tEngineCurve;

typedef struct
{
    tEngineCurve	curve;
    float		revsLimiter;
    float		revsMax;
    float		tickover;
    float		I;
    float		rads;   /* revs in rad/s ... */
    float		Tq;	/* output torque */
    float       Tq_response; /* response Tq due to mismatch */
    float       I_joint; /* joint inertia */
    float		fuelcons;
    float		brakeCoeff; /* coefficient for constant engine brake */
    float		brakeLinCoeff; /* coefficient for RPM dependent engine brake */
	float       pressure;
	float       exhaust_pressure;
	float       exhaust_refract;
} tEngine;

#endif /* _ENGINE_H_ */ 



