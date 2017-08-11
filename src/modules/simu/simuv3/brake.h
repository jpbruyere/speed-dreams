/***************************************************************************

    file                 : brake.h
    created              : Sun Mar 19 00:05:34 CET 2000
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

#ifndef _BRAKE_H_
#define _BRAKE_H_

typedef struct
{
    float	pressure;
    float	Tq;
    float	coeff;
    float	I;
    float	radius;
    float	temp;
} tBrake;

typedef struct
{
    float	rep;	/* front/rear repartition */ 
    float	coeff;
    float   ebrake_pressure;
} tBrakeSyst;



#endif /* _BRAKE_H_ */ 



