/***************************************************************************

    file                 : mainmenu.h
    created              : Sat Mar 18 23:42:51 CET 2000
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


#pragma once

#if _WIN32
#include <windows.h>
#endif

extern void *menuHandle;

// What's this ? RacemanModLoaded never set anywhere but initialized to 0 !
//extern tModList *RacemanModLoaded;

extern int PreviewMenuInit(const char *pFile);
extern int PreviewMenuRun(void);


