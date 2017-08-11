/***************************************************************************

    file                 : aero.h
    created              : Sun Mar 19 00:04:59 CET 2000
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

#ifndef _AERO_H_
#define _AERO_H_

typedef struct
{
    /* dynamic */
    float	drag;		/* drag force along car x axis */
    float	lift[2];	/* front & rear lift force along car z axis */

    /* static */
    float	SCx2;
    float	Clift[2];	/* front & rear lift due to body not wings */
    float	Cd;		/* for aspiration */
} tAero;


typedef struct
{
    /* dynamic */
    glm::vec3	forces;
    float	Kx;
    float	Kz;
	float	angle;
    
    /* static */
    glm::vec3	staticPos;
    
} tWing;



#endif /* _AERO_H_  */ 



