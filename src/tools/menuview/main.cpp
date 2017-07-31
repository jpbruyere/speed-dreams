/***************************************************************************

    file                 : main.cpp
    created              : Sat Sep  2 10:40:47 CEST 2000
    copyright            : (C) 2000 by Patrice & Eric Espie
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

#include <string>

#ifdef _MSC_VER
#include <windows.h>
#endif

#include <portability.h>

#include <tgfclient.h>

#include "previewmenu.h"


class Application : public GfuiApplication
{
 public:

	//! Constructor.
	Application()
    : GfuiApplication("MenuView", "<no version>", "XML menu viewer")
	{
    }

    //! Initialization.
    virtual void initialize(bool bLoggingEnabled, int argc = 0, char **argv = 0)
    {
        // Base initialization first.
        GfuiApplication::initialize(bLoggingEnabled, argc, argv);

        // Help about the specific options.
		addOptionsHelpSyntaxLine("<menu file>");
        addOptionsHelpExplainLine("- <menu file> : the menu XML file to load");
	}

	//! Parse the command line options.
	bool parseOptions()
	{
		// First the standard ones.
		if (!GfuiApplication::parseOptions())
			return false;

		// Then the specific ones.
		if (!_vecRemArgs.empty())
		{
			_strMenuFile = _vecRemArgs.front();
		}
		else
		{
			printUsage("No file specified.");
			return false;
		}

		// If "data dir" specified in any way, cd to it.
		if(chdir(GfDataDir()))
		{
			GfLogError("Could not start %s : failed to cd to the datadir '%s' (%s)\n",
					   name().c_str(), GfDataDir(), strerror(errno));
			return false;
		}

		return true;
	}

	//! Activate the GUI.
	void showMenu()
	{
		PreviewMenuInit(_strMenuFile.c_str());
		PreviewMenuRun();
	}

 private:

	//! The menu XML descriptor file to load.
	std::string _strMenuFile;
};

int main(int argc, char *argv[])
{
	// Create the MenuView application
	Application app;
	app.initialize(/*bLoggingEnabled=*/true, argc, argv);

	// Parse the command line options
    if (!app.parseOptions())
		return 1;

	// Update user settings files from installed ones.
    app.updateUserSettings();

    // Initialize the event loop management layer.
	GfuiEventLoop* pEventLoop = new GfuiEventLoop;
	app.setEventLoop(pEventLoop);

	// Setup the window / screen and menu infrastructure (needs an event loop).
    if (!app.setupWindow())
		return 1;

	// Display the menu.
	app.showMenu();

	// App. event loop.
	app.eventLoop()();

	// That's all.
	return 0;
}

