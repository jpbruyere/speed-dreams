/***************************************************************************

    file        : racemessage.h
    created     : Sat Nov 23 09:35:21 CET 2002
    copyright   : (C) 2002 by Eric Espie                        
    email       : eric.espie@torcs.org   
    version     : $Id$                                  

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
    	    Race message management. Don't use directly, call ReSituation::setRaceMessage.
    @author	<a href=mailto:torcs@free.fr>Eric Espie</a>
    @version	$Id$
*/

#ifndef _RACEMESSAGE_H_
#define _RACEMESSAGE_H_

struct RmInfo;

extern void ReRaceMsgManage(struct RmInfo* pReInfo);
extern void ReRaceMsgSet(struct RmInfo* pReInfo, const char *msg, double life = -1);
extern void ReRaceMsgSetBig(struct RmInfo* pReInfo, const char *msg, double life = -1);


#endif /* _RACEMESSAGE_H_ */ 



