#!/usr/bin/env python
# -*- coding: utf-8 -*-


"""Application entry point.

A demo. app. for testing CEGUI as a future possible GUI library for Speed Dreams,
built on top of PyCEGUI and PyOpenGL.

"""

# Import: std
import sys, os

# Import: User
from errors import InitializationError
from input import Input
from video import Video
from gui import GUI

# Import: Configuration
from configuration import TheConfig


# Main
def main():

	gfx = Video()
	inp = Input()
	gui = GUI()

	# Initialize
	try:
		gfx.initialize()
		inp.initialize()
		gui.initialize()
	except InitializationError as error:
		print(error)
		return 1

	# Setup the interface
	gui.setupInterface()

	# Main Loop
	gfx.enterMainLoop()

	# Done
	# - We will never actually get here.
	gfx.shutdown()
	
	return 0

# Guard
if __name__ == '__main__':

	# Set current directory to the install one if specified
	# (resource files are identified by a relative path to the current folder) ;
	# otherwise assume we are already in it.
	instDir = os.path.dirname(sys.argv[0])
	if instDir:
		os.chdir(instDir)

	# Parse command line args.
	for arg in sys.argv:
		if arg in ('-l', '--layout'):
			TheConfig.useLayouts = True
		if arg in ('-c', '--config'):
			TheConfig.useConfigFile = True

	sys.exit(main())
