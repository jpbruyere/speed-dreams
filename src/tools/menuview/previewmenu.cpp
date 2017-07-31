/***************************************************************************

    file                 : mainmenu.cpp
    created              : Sat Mar 18 23:42:38 CET 2000
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


#include <cstdio>

#include <tgfclient.h>
#include <guimenu.h>

#include "previewmenu.h"


void *menuHandle = NULL;

std::string g_strFile;

void LoadMenuScreen();

static void
onQuit(void * /* dummy */)
{
    GfuiApp().eventLoop().postQuit();
}

static void
PreviewMenuActivate(void * /* dummy */)
{
}

int ReadControl(void *menuHdle, void *param, std::string strType, const char *pControlName)
{
	if (strType == GFMNU_TYPE_TEXT_BUTTON || strType == GFMNU_TYPE_IMAGE_BUTTON)
		return GfuiMenuCreateButtonControl(menuHandle,param,pControlName,0,NULL);
	else if (strType == GFMNU_TYPE_EDIT_BOX)
		return GfuiMenuCreateEditControl(menuHandle,param,pControlName,0,NULL,NULL);
	else if (strType == GFMNU_TYPE_LABEL)
		return GfuiMenuCreateLabelControl(menuHandle,param,pControlName);
	else if (strType == GFMNU_TYPE_STATIC_IMAGE)
		return GfuiMenuCreateStaticImageControl(menuHandle,param,pControlName);
	else if (strType == GFMNU_TYPE_COMBO_BOX)
		return GfuiMenuCreateComboboxControl(menuHandle,param,pControlName,0,NULL);
	else if (strType == GFMNU_TYPE_SCROLL_LIST)
	{
		const int id = GfuiMenuCreateScrollListControl(menuHandle,param,pControlName,0,NULL);
		GfuiScrollListInsertElement(menuHdle, id, "Item 1", 0, NULL);
		GfuiScrollListInsertElement(menuHdle, id, "Item 2", 0, NULL);
		GfuiScrollListInsertElement(menuHdle, id, "Item 3", 0, NULL);
		return id;
	}
	else if (strType == GFMNU_TYPE_CHECK_BOX)
		return GfuiMenuCreateCheckboxControl(menuHandle,param,pControlName,0,NULL);
	else if (strType == GFMNU_TYPE_PROGRESS_BAR)
		return GfuiMenuCreateProgressbarControl(menuHandle,param,pControlName);

	return -1;
}

void ShowDynamicControls(void *menuHdle, void *param)
{

	if (GfParmListSeekFirst(param, GFMNU_SECT_DYNAMIC_CONTROLS) == 0)
	{
		do
		{
			std::string strControlName =
				GfParmListGetCurEltName (param, GFMNU_SECT_DYNAMIC_CONTROLS);
			std::string strType = GfParmGetCurStr(param, GFMNU_SECT_DYNAMIC_CONTROLS, "type", "");
			ReadControl(menuHdle, param,strType,strControlName.c_str());
		} while (GfParmListSeekNext(param, GFMNU_SECT_DYNAMIC_CONTROLS) == 0);
	}
}

void
ReloadMenuScreen(void *)
{
	GfuiScreenDeactivate();
	LoadMenuScreen();
	PreviewMenuRun();
}

void
LoadMenuScreen()
{
    menuHandle = GfuiScreenCreate((float*)NULL,
								  NULL, PreviewMenuActivate,
								  NULL, (tfuiCallback)NULL,
								  1);

	void *param = GfParmReadFile(g_strFile.c_str(), GFPARM_RMODE_REREAD);
	if (!param)
		param = GfParmReadFileLocal(g_strFile.c_str(), GFPARM_RMODE_REREAD);

    GfuiMenuCreateStaticControls(menuHandle, param);

	ShowDynamicControls(menuHandle, param);

	GfuiAddKey(menuHandle, GFUIK_F5, "Re-load", NULL, ReloadMenuScreen, NULL);
    GfuiAddKey(menuHandle, 'Q', "Quit", 0, onQuit, NULL);
    GfuiAddKey(menuHandle, 'q', "Quit", 0, onQuit, NULL);
    GfuiAddKey(menuHandle, GFUIK_ESCAPE, "Quit", 0, onQuit, NULL);

	GfParmReleaseHandle(param);
}

int
PreviewMenuInit(const char* pFile)
{
	g_strFile = pFile;
	LoadMenuScreen();
    return 0;
}
/*
 * Function
 *
 *
 * Description
 *
 *
 * Parameters
 *
 *
 * Return
 *
 *
 * Remarks
 *
 */
int
PreviewMenuRun(void)
{
    GfuiScreenActivate(menuHandle);
    return 0;
}
