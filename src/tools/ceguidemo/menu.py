#!/usr/bin/env python
# -*- coding: utf-8 -*-


"""Menu.

Base class for any menu.

"""

# Import: std
import sys

# Import: OpenGL
from OpenGL.GLUT import *

# Import: PyCEGUI
import PyCEGUI

# Import: User
from errors import InitializationError


# Menu
class Menu(object):

	# Initialize
	def initialize(self, name, title=None, layout=None, background=None):
		
		# If specified, load the layout.
		if layout:

			window = PyCEGUI.WindowManager.getSingleton().loadWindowLayout(layout + ".layout")

		# Otherwise, build it up through code.
		else:

			# 1) Menu root window = background image.
			winMgr = PyCEGUI.WindowManager.getSingleton()
			if background:

				window = winMgr.createWindow('CEGUIDemo/StaticImage', name)

				# 1.a) Disable standard background
				window.setProperty("BackgroundEnabled", "false")

				# 1.b) Set the background image
				window.setProperty("Image", "set:%s image:full_image" % background)

			else:

				window = winMgr.createWindow('CEGUIDemo/FrameWindow', name)

			# 1.d) Set area rectangle
			window.setArea(PyCEGUI.UDim(0, 0), PyCEGUI.UDim(0, 0),
						   PyCEGUI.UDim(1, 0), PyCEGUI.UDim(1, 0))

			# 1.e) Disable frame and standard background
			window.setProperty("FrameEnabled", "false")

			# 2) Logo image
			imgSetMgr = PyCEGUI.ImagesetManager.getSingleton()
			imgSetMgr.createFromImageFile("Logo", "sd-logo.png")
			imgLogo = winMgr.createWindow('CEGUIDemo/StaticImage', name + "/ImgLogo")
			imgLogo.setArea(PyCEGUI.UDim(0, 0), PyCEGUI.UDim(0, 0),
						 PyCEGUI.UDim(0.3, 0), PyCEGUI.UDim(0.25, 0))
			imgLogo.setProperty("FrameEnabled", "false")
			imgLogo.setProperty("BackgroundEnabled", "false")
			imgLogo.setProperty("Image", "set:Logo image:full_image")

			window.addChildWindow(imgLogo)

			# 3) Title
			txtTitle = winMgr.createWindow('CEGUIDemo/PageTitle', name + "/TxtTitle")
			txtTitle.setArea(PyCEGUI.UDim(0.2, 0), PyCEGUI.UDim(0.1, 0),
							 PyCEGUI.UDim(0.7, 0), PyCEGUI.UDim(0.1, 0))
			txtTitle.setText(title or "<undefined title>")
			txtTitle.setTooltipText("Yeah, this is the title of the menu !")
			txtTitle.setProperty("Font", "MenuTitle")
			txtTitle.setProperty("FrameEnabled", "false")
			txtTitle.setProperty("BackgroundEnabled", "true")

			window.addChildWindow(txtTitle)

			# 4) Frame rate indicator
			txtFrameRate = winMgr.createWindow('CEGUIDemo/StaticText', name + "/TxtFrameRate")
			txtFrameRate.setArea(PyCEGUI.UDim(0.95, 0), PyCEGUI.UDim(0.01, 0),
							  PyCEGUI.UDim(0.04, 0), PyCEGUI.UDim(0.03, 0))
			txtFrameRate.setText("--.-")
			txtFrameRate.setTooltipText("Frame rate (F/s)")
			txtFrameRate.setProperty("Font", "TextSmall")
			txtFrameRate.setProperty("FrameEnabled", "false")
			txtFrameRate.setProperty("BackgroundEnabled", "false")
			txtFrameRate.setProperty("HorzFormatting", "RightAligned")
			txtFrameRate.setProperty("VertFormatting", "TopAligned")
			txtFrameRate.setAlwaysOnTop(True);

			window.addChildWindow(txtFrameRate)

		# Retrieve the root window and its children.
		self.window = window
		self.imgLogo = window.getChild(name + "/ImgLogo")
		self.txtTitle = window.getChild(name + "/TxtTitle")
		self.txtFrameRate = window.getChild(name + "/TxtFrameRate")

		# Trace info. about children.
		#print("Menu: Children (n=%d) :" % self.window.getChildCount())
		#for chldInd in range(self.window.getChildCount()):
		#	print("  #%d : name=%r" % (chldInd, self.window.getChildAtIdx(chldInd).getName()))

		# Setup animations.
		self.animFadeIn = PyCEGUI.AnimationManager.getSingleton().instantiateAnimation("MenuFadeIn")
		self.animFadeIn.setTargetWindow(self.window)
		#self.animFadeIn.start()
		#self.animFadeOut = PyCEGUI.AnimationManager.getSingleton().instantiateAnimation("MenuFadeOut")
		#self.animFadeOut.setTargetWindow(self.window)
		#self.animFadeOut.start()

		return window

	# - Wrapper method to define the subscription/listener relationships.
	# - If there are a lot, it may behoove the coder to encapsulate them in methods, then call those methods here.
	def connectHandlers(self):

		# Event subscriptions :
		# * keyboard.
		self.window.subscribeEvent(PyCEGUI.Window.EventKeyDown, self, 'onKeyDown')
			
		# * window update (for the frame rate indicator).
		self.window.subscribeEvent(PyCEGUI.Window.EventWindowUpdated, self, 'onUpdate')
		
	#def disconnectHandlers(self):

	#	self.window.removeEvent(PyCEGUI.Window.EventKeyDown)
	#	self.window.removeEvent(PyCEGUI.Window.EventWindowUpdated)
		
	# Setup
	def setup(self):

		self.connectHandlers()

		# Debug : Seems that EventActivated is only fired once ever !
		self.window.subscribeEvent(PyCEGUI.Window.EventActivated, self, 'onActivated')
		
		self.window.subscribeEvent(PyCEGUI.Window.EventActivated, self, 'onDeactivated')

	# Activate
	def deactivate(self):

		print("%s.deactivate" % self.__class__.__name__)
		
		# Detach.
		#self.disconnectHandlers()
		self.window.setMutedState(True)
		self.window.hide()
		self.window.deactivate()
	
	# Activate
	def activate(self, previous=None):
	
		print("%s.activate" % self.__class__.__name__)

		# Attach new.
		PyCEGUI.System.getSingleton().setGUISheet(self.window)
		#self.connectHandlers()
		self.window.setMutedState(False)
		self.window.show()
		self.window.activate() # Set focus (needed for EventKeyDown being received).

		# Save previous menu if specified (in case we need to return to this one).
		if previous:
			self.prevMenu = previous

	# Return to the previous menu.
	def switchTo(self, menu):
	
		self.deactivate()
		menu.activate(previous=self)

	# Return to the previous menu.
	def back(self):
	
		if self.prevMenu:
			self.deactivate()
			self.prevMenu.activate()
		else:
			print("Warning: No previous menu to return to ; ignoring.")

	# Update frame rate indicator.
	def onUpdate(self, winArgs): # Bug: Not an UpdateEventArgs, but a WindowEventArgs !
	
		self.currTime = glutGet(GLUT_ELAPSED_TIME)
		elapsed = (self.currTime - self.lastTime) / 1000.0
		self.nFrames += 1
		if elapsed >= 1.0: # Skidding mean for each second
			self.txtFrameRate.setText("%4.1f" % (self.nFrames / elapsed))
			self.nFrames = 0
			self.lastTime = self.currTime

	def onKeyDown(self, keyArgs):
		
		# Just in case not specialised in actual class.
		print("Menu.onKeyDown: sc=", keyArgs.scancode)
		return False
		
	# Debug : Seems that EventActivated is only fired once ever !
	def onActivated(self, args):
			
		print("%s.onActivated" % self.__class__.__name__)

		#self.connectHandlers()

		# Initialize frame rate data.
		self.nFrames = 0
		self.currTime = self.lastTime = glutGet(GLUT_ELAPSED_TIME)
		
		return False

	def onDeactivated(self, args):
			
		print("%s.onDeactivated" % self.__class__.__name__)
		#self.disconnectHandlers()
		return False
