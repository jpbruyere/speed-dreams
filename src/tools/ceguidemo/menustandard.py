#!/usr/bin/env python
# -*- coding: utf-8 -*-


"""Standard menu (abstract class)

Only here for gathering the widgets which are common to "standard" menus :
top buttons looking like if there was a classical pull-down menu

"""

# Import: std
import sys

# Import: PyCEGUI
import PyCEGUI

# Import: Menus and dialogs
from menu import Menu
from dialogs import DialogOKCancel

# Import: MenuManager
from menumanager import MenuManager


# Standard menu
class MenuStandard(Menu):

	def __init__(self):
	
		Menu.__init__(self)

	# Initialize
	def initialize(self, name, title=None, layout=None, background=None):

		# Standard initialization.
		window = Menu.initialize(self, name=name, title=title, layout=layout, background=background)

		# If no layout specified, go on building up the menu through code.
		if not layout:

			# Specific to these menus.
			btnCredits = PyCEGUI.WindowManager.getSingleton().createWindow("CEGUIDemo/Button", name + "/BtnCredits")
			btnCredits.setText("Credits")
			btnCredits.setTooltipText("Thanks to all contributors !")
			btnCredits.setXPosition(PyCEGUI.UDim(0.3, 0.0))
			btnCredits.setYPosition(PyCEGUI.UDim(0.0, 0.0))
			btnCredits.setWidth(PyCEGUI.UDim(0.13, 0.0))
			btnCredits.setHeight(PyCEGUI.UDim(0.05, 0.0))
			btnCredits.setProperty("Font", "MenuMedium")

			window.addChildWindow(btnCredits)

			# Specific to these menus.
			btnProfiles = PyCEGUI.WindowManager.getSingleton().createWindow("CEGUIDemo/Button", name + "/BtnProfiles")
			btnProfiles.setText("Profiles")
			btnProfiles.setTooltipText("Configure player input controls / profiles")
			btnProfiles.setXPosition(PyCEGUI.UDim(0.45, 0.0))
			btnProfiles.setYPosition(PyCEGUI.UDim(0.0, 0.0))
			btnProfiles.setWidth(PyCEGUI.UDim(0.13, 0.0))
			btnProfiles.setHeight(PyCEGUI.UDim(0.05, 0.0))
			btnProfiles.setProperty("Font", "MenuMedium")

			window.addChildWindow(btnProfiles)

			# Specific to these menus.
			btnOptions = PyCEGUI.WindowManager.getSingleton().createWindow("CEGUIDemo/Button", name + "/BtnOptions")
			btnOptions.setText("Options")
			btnOptions.setTooltipText("Settings for display, graphics, simulation ...")
			btnOptions.setXPosition(PyCEGUI.UDim(0.60, 0.0))
			btnOptions.setYPosition(PyCEGUI.UDim(0.0, 0.0))
			btnOptions.setWidth(PyCEGUI.UDim(0.13, 0.0))
			btnOptions.setHeight(PyCEGUI.UDim(0.05, 0.0))
			btnOptions.setProperty("Font", "MenuMedium")

			window.addChildWindow(btnOptions)

			# Specific to these menus.
			btnExit = PyCEGUI.WindowManager.getSingleton().createWindow("CEGUIDemo/Button", name + "/BtnExit")
			btnExit.setText("Exit")
			btnExit.setTooltipText("Exit from the game")
			btnExit.setXPosition(PyCEGUI.UDim(0.75, 0.0))
			btnExit.setYPosition(PyCEGUI.UDim(0.0, 0.0))
			btnExit.setWidth(PyCEGUI.UDim(0.13, 0.0))
			btnExit.setHeight(PyCEGUI.UDim(0.05, 0.0))
			btnExit.setProperty("Font", "MenuMedium")

			window.addChildWindow(btnExit)

		# Retrieve the root window children created here.
		self.btnCredits = window.getChild(name + "/BtnCredits")
		self.btnProfiles = window.getChild(name + "/BtnProfiles")
		self.btnOptions = window.getChild(name + "/BtnOptions")
		self.btnExit = window.getChild(name + "/BtnExit")

		return window

	# connectHandlers
	# - Wrapper method to define the subscription/listener relationships.
	# - If there are a lot, it may behoove the coder to encapsulate them in methods, then call those methods here.
	def connectHandlers(self):

		# Inherited connections.
		Menu.connectHandlers(self)

		# Specific connections.
		self.btnCredits.subscribeEvent(PyCEGUI.PushButton.EventClicked, self, "onCreditsButtonClicked")
		self.btnProfiles.subscribeEvent(PyCEGUI.PushButton.EventClicked, self, "onProfilesButtonClicked")
		self.btnOptions.subscribeEvent(PyCEGUI.PushButton.EventClicked, self, "onOptionsButtonClicked")
		self.btnExit.subscribeEvent(PyCEGUI.PushButton.EventClicked, self, "onExitButtonClicked")

	# Handlers
	def onCreditsButtonClicked(self, args):

		self.switchTo(MenuManager.get("Credits"))

	def onProfilesButtonClicked(self, args):

		self.switchTo(MenuManager.get("Profiles"))

	def onOptionsButtonClicked(self, args):

		self.switchTo(MenuManager.get("Options"))

	def onExitButtonClicked(self, args):

		DialogOKCancel.instance().show(self.window, self.onOKCancelClosed, "Really quit ?")

	def onOKCancelClosed(self, reallyQuit=False):

		if reallyQuit:
			print("Exiting (on exit button) ...")
			sys.exit(0)

	def onKeyDown(self, keyArgs):

		print("MenuStandard.onKeyDown: sc=%d, " % keyArgs.scancode)
		if keyArgs.scancode == PyCEGUI.Key.Escape:

			keyArgs.handled = True
			self.onExitButtonClicked(keyArgs)
			#print("Exiting (on Escape key) ...")
			#sys.exit(0)

			return True

		return False
