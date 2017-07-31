#!/usr/bin/env python
# -*- coding: utf-8 -*-


"""Main menu.


"""

# Import: std
import sys

# Import: PyCEGUI
import PyCEGUI

# Import: Configuration
from configuration import TheConfig

# Import: Menus
from menustandard import MenuStandard
from menumanager import MenuManager

# Main menu
class MenuMain(MenuStandard):

	# Singleton pattern.
	singleton = None

	def instance():
	
		if not MenuMain.singleton:
			MenuMain.singleton = MenuMain()
			MenuMain.singleton.initialize()
			MenuMain.singleton.setup()
			
		return MenuMain.singleton

	instance = staticmethod(instance)

	def __init__(self):

		MenuStandard.__init__(self)

	# Initialize
	def initialize(self):

		name = "MenuMain"

		# Use layout if specified.
		if TheConfig.useLayouts:
			
			window = MenuStandard.initialize(self, name=name, title="Welcome", layout="menumain")
			
		else:
			
			# If no layout specified, go on building up the menu through code.
			window = MenuStandard.initialize(self, name=name, title="Welcome", background="SplashMain")

			# Specific to this menu.
			btnPractice = PyCEGUI.WindowManager.getSingleton().createWindow("CEGUIDemo/MainButton", name + "/BtnPractice")
			btnPractice.setText("Practice")
			btnPractice.setTooltipText("A free practice session for one player on one track")
			btnPractice.setXPosition(PyCEGUI.UDim(0.22, 0.0))
			btnPractice.setYPosition(PyCEGUI.UDim(0.3, 0.0))
			btnPractice.setWidth(PyCEGUI.UDim(0.6, 0.0))
			btnPractice.setHeight(PyCEGUI.UDim(0.07, 0.0))
			btnPractice.setProperty("Font", "MenuBig")

			window.addChildWindow(btnPractice)

			btnQuickRace = PyCEGUI.WindowManager.getSingleton().createWindow("CEGUIDemo/MainButton", name + "/BtnQuickRace")
			btnQuickRace.setText("QuickRace")
			btnQuickRace.setTooltipText("A race on one track, with no qualifying session")
			btnQuickRace.setXPosition(PyCEGUI.UDim(0.22, 0.0))
			btnQuickRace.setYPosition(PyCEGUI.UDim(0.37, 0.0))
			btnQuickRace.setWidth(PyCEGUI.UDim(0.6, 0.0))
			btnQuickRace.setHeight(PyCEGUI.UDim(0.07, 0.0))
			btnQuickRace.setProperty("Font", "MenuBig")

			window.addChildWindow(btnQuickRace)

			btnSingleEvent = PyCEGUI.WindowManager.getSingleton().createWindow("CEGUIDemo/MainButton", name + "/BtnSingleEvent")
			btnSingleEvent.setText("SingleEvent")
			btnSingleEvent.setTooltipText("A race on one track, with various qualifying schemes")
			btnSingleEvent.setXPosition(PyCEGUI.UDim(0.22, 0.0))
			btnSingleEvent.setYPosition(PyCEGUI.UDim(0.44, 0.0))
			btnSingleEvent.setWidth(PyCEGUI.UDim(0.6, 0.0))
			btnSingleEvent.setHeight(PyCEGUI.UDim(0.07, 0.0))
			btnSingleEvent.setProperty("Font", "MenuBig")
			#btnSingleEvent.setEnabled(False)

			window.addChildWindow(btnSingleEvent)

			cbxSingleEvent = PyCEGUI.WindowManager.getSingleton().createWindow("CEGUIDemo/Combobox", name + "/CbxSingleEvent.SubType")
			cbxSingleEvent.setTooltipText("Select the type of single event race")
			cbxSingleEvent.setXPosition(PyCEGUI.UDim(0.40, 0.0))
			cbxSingleEvent.setYPosition(PyCEGUI.UDim(0.50, 0.0))
			cbxSingleEvent.setWidth(PyCEGUI.UDim(0.42, 0.0))
			cbxSingleEvent.setHeight(PyCEGUI.UDim(0.16, 0.0))
			cbxSingleEvent.setProperty("Font", "MenuMedium")

			window.addChildWindow(cbxSingleEvent)

		# Retrieve the window descendants created here.
		self.btnPractice = window.getChild(name + "/BtnPractice")
		self.btnQuickRace = window.getChild(name + "/BtnQuickRace")
		self.btnSingleEvent = window.getChild(name + "/BtnSingleEvent")
		self.cbxSingleEvent = window.getChild(name + "/CbxSingleEvent.SubType")

		# Complete widget initialization (whatever creation mode : code or .layout).
		self.lstItemsSingleEvent = []
		for subType in ("Endurance", "Challenge", "1936 Grand Prix", "LS1 Challenge"):
			cbxItem = PyCEGUI.ListboxTextItem(subType)
			cbxItem.setSelectionBrushImage("CEGUIDemo", "ComboboxSelectionBrush")
			cbxItem.setSelectionColours(0xFF3FFFEE)
			self.cbxSingleEvent.addItem(cbxItem)
			self.lstItemsSingleEvent.append(cbxItem) # Avoid its being GC'd at return !

		itemCbx = self.cbxSingleEvent.findItemWithText("Challenge", None) # Get the "Challenge" item
		itemCbx.setSelected(True) # Select this item
		self.cbxSingleEvent.setText(itemCbx.getText()) # Copy the item's text into the Editbox

		return window
		
	# connectHandlers
	# - Wrapper method to define the subscription/listener relationships.
	# - If there are a lot, it may behoove the coder to encapsulate them in methods, then call those methods here.
	def connectHandlers(self):

		# Inherited connections.
		MenuStandard.connectHandlers(self)

		# Specific connections.
		self.btnPractice.subscribeEvent(PyCEGUI.PushButton.EventClicked, self, "onPracticeButtonClicked")
		self.btnQuickRace.subscribeEvent(PyCEGUI.PushButton.EventClicked, self, "onQuickRaceButtonClicked")
		self.btnSingleEvent.subscribeEvent(PyCEGUI.PushButton.EventClicked, self, "onSingleEventButtonClicked")

	# Handler: buttonClicked
	def onPracticeButtonClicked(self, args):

		self.switchTo(MenuManager.get("TrackSelect"))

	def onQuickRaceButtonClicked(self, args):

		self.switchTo(MenuManager.get("TrackSelect"))

	def onSingleEventButtonClicked(self, args):

		self.switchTo(MenuManager.get("TrackSelect"))
