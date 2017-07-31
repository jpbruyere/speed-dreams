#!/usr/bin/env python
# -*- coding: utf-8 -*-


"""Options menu.


"""

# Import: std
import sys

# Import: PyCEGUI
import PyCEGUI

# Import: Configuration
from configuration import TheConfig

# Import: Menu
from menu import Menu

# Options menu
class MenuOptions(Menu):

	singleton = None

	def instance():
	
		if not MenuOptions.singleton:
			MenuOptions.singleton = MenuOptions()
			MenuOptions.singleton.initialize()
			MenuOptions.singleton.setup()
			
		return MenuOptions.singleton

	instance = staticmethod(instance)

	def __init__(self):

		Menu.__init__(self)
	
	# Initialize
	def initialize(self):

		name = "MenuOptions"
		
		# No code written for this menu : use mandatory layout.
		window = Menu.initialize(self, name=name, title="Options", layout="menuoptions")

		# Retrieve window descendants created here.
		self.btnAccept = window.getChild(name + "/BtnAccept")
		self.btnCancel = window.getChild(name + "/BtnCancel")
		self.cbxWinSize = window.getChild(name + "/CbxWindowSize")
		
		# Complete widget initialization.
		self.cbxWinSizeItems = []
		for size in (" 800 x  512", "1024 x  640", "1280 x  800", "1680 x 1050"):
			cbxItem = PyCEGUI.ListboxTextItem(size)
			cbxItem.setSelectionBrushImage("CEGUIDemo", "ComboboxSelectionBrush")
			cbxItem.setSelectionColours(0xFF3FFFEE)
			self.cbxWinSize.addItem(cbxItem)
			self.cbxWinSizeItems.append(cbxItem) # Avoid its being GC'd at return !
			if size.startswith("1024"):
				self.cbxWinSize.setText(cbxItem.getText())

		# TODO.

		return window

	# connectHandlers
	# - Wrapper method to define the subscription/listener relationships.
	# - If there are a lot, it may behoove the coder to encapsulate them in methods, then call those methods here.
	def connectHandlers(self):

		# Inherited connections.
		Menu.connectHandlers(self)

		# Specific connections.
		self.btnCancel.subscribeEvent(PyCEGUI.PushButton.EventClicked, self, "onCancelButtonClicked")
		self.btnAccept.subscribeEvent(PyCEGUI.PushButton.EventClicked, self, "onAcceptButtonClicked")

	def onCancelButtonClicked(self, args):

		print("Options : Cancelled.")
		self.back()

	def onAcceptButtonClicked(self, args):

		print("Options : Accepted.")
		self.back()

	def onKeyDown(self, keyArgs):

		if keyArgs.scancode == PyCEGUI.Key.Escape:
			keyArgs.handled += 1
			self.onCancelButtonClicked(keyArgs)
			return True
		elif keyArgs.scancode == PyCEGUI.Key.Return:
			keyArgs.handled += 1
			self.onAcceptButtonClicked(keyArgs)
			return True

		return False
