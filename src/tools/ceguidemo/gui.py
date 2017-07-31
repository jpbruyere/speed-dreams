#!/usr/bin/env python
# -*- coding: utf-8 -*-


"""GUI.

This class is the entry point for the GUI; which is to say that it starts the
main menu and the rest is event driven.

"""

# Import: std
import sys

# Import: PyCEGUI
import PyCEGUI

# Import: User
from errors import InitializationError
from menufactory import MenuFactory
from menumanager import MenuManager

# Import: Configuration
from configuration import TheConfig


# GUI
class GUI(object):

	# Initialize: Resources
	def initializeResources(self):
		
		if not TheConfig.useConfigFile:
		
			rp = PyCEGUI.System.getSingleton().getResourceProvider()
			rp.setResourceGroupDirectory('schemes', './datafiles/schemes')
			rp.setResourceGroupDirectory('imagesets', './datafiles/imagesets')
			rp.setResourceGroupDirectory('fonts', './datafiles/fonts')
			rp.setResourceGroupDirectory('layouts', './datafiles/layouts')
			rp.setResourceGroupDirectory('looknfeels', './datafiles/looknfeels')
			rp.setResourceGroupDirectory('animations', './datafiles/animations')
			rp.setResourceGroupDirectory('schemas', './datafiles/xml_schemas')

			PyCEGUI.Imageset.setDefaultResourceGroup('imagesets')
			PyCEGUI.Font.setDefaultResourceGroup('fonts')
			PyCEGUI.Scheme.setDefaultResourceGroup('schemes')
			PyCEGUI.WidgetLookManager.setDefaultResourceGroup('looknfeels')
			PyCEGUI.WindowManager.setDefaultResourceGroup('layouts')
			PyCEGUI.AnimationManager.setDefaultResourceGroup('animations')

		# Doesn't seem actually useful ...
		#parser = PyCEGUI.System.getSingleton().getXMLParser()
		#if parser.isPropertyPresent('SchemaDefaultResourceGroup'):
		#	parser.setProperty('SchemaDefaultResourceGroup', 'schemas')

	# Initialize: Defaults
	def initializeDefaults(self):
		
		if not TheConfig.useConfigFile:
		
			sm = PyCEGUI.SchemeManager.getSingleton()
			sm.create('ceguidemo.scheme')
			PyCEGUI.System.getSingleton().setDefaultMouseCursor('CEGUIDemo', 'MouseArrow')
			PyCEGUI.System.getSingleton().setDefaultTooltip('CEGUIDemo/Tooltip')
			PyCEGUI.System.getSingleton().setDefaultFont('MenuNormal')
		
		# Can't get this done through XML (.scheme)'
		PyCEGUI.AnimationManager.getSingleton().loadAnimationsFromXML('animations.xml')

	# Initialize
	def initialize(self):
		
		try:
			
			self.initializeResources()
			self.initializeDefaults()
			
			MenuFactory.initialize()
		
		except Exception, msg:
			
			raise InitializationError(msg)

	# Setup: Interface
	def setupInterface(self):

		MenuManager.get("Main").activate()
