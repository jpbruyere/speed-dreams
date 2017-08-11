/***************************************************************************

    file                 : aero.h
    created              : Sun Mar 19 00:04:59 CET 2000
    copyright            : (C) 2000 by Eric Espie
    email                : torcs@free.fr
    version              : $Id: aero.h 2917 2010-10-17 19:03:40Z pouillot $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef _AERO_H_
#define _AERO_H_

typedef struct
{
    /* dynamic */
    float	drag;		/* drag force along car x axis */
    float	lift[2];	/* front & rear lift force along car z axis */

    /* static */
    //float	SCx2;       replaced by CdBody as initial Cd
    float	Clift[2];	/* front & rear lift due to body not wings */
    float	Cd;		    /* for aspiration, value updated depending on wing angles */
    float	CdBody;	    /* for aspiration, value without wings, for variable wing angles */
} tAero;


typedef struct
{
    /* dynamic */
    glm::vec3	forces;
    float	Kx;
    float	Kz;
    float	Kz_org;
    float	angle;
    
    /* static */
    glm::vec3	staticPos;

//>>> simuV4
    /* static for wings with const angle */
    float	AoAatMax;	/* [deg] Angle of Attack at the maximum of coefficient of lift */
    float	AoAatZero;	/* [deg] Angle of Attack at coefficient of lift = 0 (-30 < AoAatZero < 0) */
    float	AoAatZRad;	/* [rad] Angle of Attack at coefficient of lift = 0 (-30 < AoAatZero < 0) */
    float	AoAOffset;	/* [deg] Offset for Angle of Attack */

    float	CliftMax;	/* Maximum of coefficient of lift (0 < CliftMax < 4) */
    float	CliftZero;	/* Coefficient of lift at Angle of Attack = 0 */
    float	CliftAsymp;	/* Asymptotic coefficient of lift at large Angle of Attack */
    float	a;			/* [deg] Scaled angle at decreasing */
    float	b;			/* Delay of decreasing */
    float	c;			/* Curvature of start of decreasing */
    float	d;			/* Scale at AoA = 0 */
    float	f;			/* Scale factor */
    
    /* parameters for THIN wing model */
    float   AoStall;    /* angle of stall =15 deg (1 < AoStall < 45 in degrees) */
    float   Stallw;     /* stall width =2 deg, (1 < Stallw < AoStall) */
    float   AR;         /* effective aspect ratio of wing, 0 means infinite, must be positive */
    float   Kx1, Kx2, Kx3, Kx4;
    float   Kz1, Kz2, Kz3;

    int		WingType;	/* -1=None, 0=FLAT, 1=PROFILE, 2=THIN... */
//<<< simuV4
} tWing;

#endif /* _AERO_H_  */ 



