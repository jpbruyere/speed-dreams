#!/usr/bin/env python
# -*- coding: utf-8 -*-


"""Results menu.


"""

# Import: std
import sys

# Import: PyCEGUI
import PyCEGUI

# Import: Configuration
from configuration import TheConfig

# Import: Menus
from menu import Menu
from menumanager import MenuManager

# Results menu
class MenuResults(Menu):

	# Static data.
	CIdRank = 0
	CIdAdvance = 1
	CIdDriver = 2
	CIdRobot = 3
	CIdCar = 4
	CIdTime = 5
	CIdBestLap = 6
	CIdNLaps = 7
	CIdTopSpeed = 8
	CIdDamages = 9
	CIdNPits = 10

	Results = \
	[ \
		(1, +2, "Yuuki Kyousou", "Simplix", "Boxer 96", "35:25:559", "1:22.336", 33, 268, 55, 2),
		(2, +0, "Marisol Carillo", "Simplix", "Spitit 300", "35:26:009", "1:23.870", 33, 255, 0, 1),
		(3, -2, "Vittorio Basso", "Simplix", "Cavvalo 360", "36:05:154", "1:24.550", 33, 262, 450, 3),
		(4, +1, "Mick Donna", "USR", "Murasama NSX", "36:45:921", "1:23.901", 33, 253, 5, 2),
		(5, -2, "Mark Duncan", "USR", "FMC GT4", "37:02:593", "1:23.568", 34, 260, 203, 3),
		(6, -4, "Don Nelson", "USR", "Lynx 220", "39:15:559", "1:20.602", 36, 285, 4500, 4),
	]

	# Singleton pattern.
	singleton = None

	def instance():
	
		if not MenuResults.singleton:
			MenuResults.singleton = MenuResults()
			MenuResults.singleton.initialize()
			MenuResults.singleton.setup()
			
		return MenuResults.singleton

	instance = staticmethod(instance)

	def __init__(self):

		Menu.__init__(self)
	
	# Initialize
	def initialize(self):

		name = "MenuResults"

		# No code written for this menu : use mandatory layout.
		window = Menu.initialize(self, name=name, title="Results", layout="menuresults")
		
		# Retrieve window descendants created here.
		self.mclTable = window.getChild(name + "/MclTable")
		self.btnContinue = window.getChild(name + "/BtnContinue")
		
		# Complete widget initialization (whatever creation mode : code or .layout).
		self.mclTable.addColumn("Rk", self.CIdRank, PyCEGUI.UDim(0.05, 0))
		self.mclTable.addColumn("Adv", self.CIdAdvance, PyCEGUI.UDim(0.05, 0))
		self.mclTable.addColumn("Driver", self.CIdDriver, PyCEGUI.UDim(0.18, 0))
		self.mclTable.addColumn("Robot", self.CIdRobot, PyCEGUI.UDim(0.10, 0))
		self.mclTable.addColumn("Car", self.CIdCar, PyCEGUI.UDim(0.17, 0))
		self.mclTable.addColumn("Time", self.CIdTime, PyCEGUI.UDim(0.12, 0))
		self.mclTable.addColumn("Best", self.CIdBestLap, PyCEGUI.UDim(0.10, 0))
		self.mclTable.addColumn("Laps", self.CIdNLaps, PyCEGUI.UDim(0.05, 0))
		self.mclTable.addColumn("Top spd", self.CIdTopSpeed, PyCEGUI.UDim(0.05, 0))
		self.mclTable.addColumn("Damages", self.CIdDamages, PyCEGUI.UDim(0.05, 0))
		self.mclTable.addColumn("Pits", self.CIdNPits, PyCEGUI.UDim(0.05, 0))

		# Note: Keep a reference to each listbox item,
		# otherwise they get garbage collected at the end of this function,
		# and then CEGUI crashes of course (see below : self.mclItems.append).
		self.mclItems = []
		for row in self.Results:
			rowId = self.mclTable.addRow()
			colId = 0
			for col in row:
				mclItem = PyCEGUI.ListboxTextItem(unicode(col))
				if colId == self.CIdAdvance:
					if col > 0:
						mclItem.setTextColours(PyCEGUI.colour(0xFFFFE000))
					elif col < 0:
						mclItem.setTextColours(PyCEGUI.colour(0xFFA0A0A0))
				self.mclTable.setItem(mclItem, colId, rowId)
				colId += 1
				self.mclItems.append(mclItem)

		return window

	# connectHandlers
	# - Wrapper method to define the subscription/listener relationships.
	# - If there are a lot, it may behoove the coder to encapsulate them in methods, then call those methods here.
	def connectHandlers(self):

		# Inherited connections.
		Menu.connectHandlers(self)

		# Specific connections.
		self.btnContinue.subscribeEvent(PyCEGUI.PushButton.EventClicked, self, "onContinueButtonClicked")

	def onContinueButtonClicked(self, args):

		self.switchTo(MenuManager.get("Main"))

	def onKeyDown(self, keyArgs):

		if keyArgs.scancode in (PyCEGUI.Key.Escape, PyCEGUI.Key.Return):
			self.onContinueButtonClicked(keyArgs)
			return True

		return False
