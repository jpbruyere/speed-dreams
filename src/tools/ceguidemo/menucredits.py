#!/usr/bin/env python
# -*- coding: utf-8 -*-


"""Credits menu.


"""

# Import: std
import sys

# Import: PyCEGUI
import PyCEGUI

# Import: Configuration
from configuration import TheConfig

# Import: Menu
from menu import Menu

# Credit menu
class MenuCredits(Menu):

	singleton = None

	def instance():
	
		if not MenuCredits.singleton:
			MenuCredits.singleton = MenuCredits()
			MenuCredits.singleton.initialize()
			MenuCredits.singleton.setup()
			
		return MenuCredits.singleton

	instance = staticmethod(instance)

	def __init__(self):

		Menu.__init__(self)
	
	# Initialize
	def initialize(self):

		name = "MenuCredits"

		# No code written for this menu : use mandatory layout.
		window = Menu.initialize(self, name=name, title="Credits", layout="menucredits")

		# Retrieve window descendants created till there.
		self.tacCred = window.getChild(name + "/TacCredits")
		self.btnBack = window.getChild(name + "/BtnBack")
		
		# Ehrrrr ... well, not much ...
		winMgr = PyCEGUI.WindowManager.getSingleton()
		tabInd = 0
		self.mclTables = []
		for tabText in ("Development Team", "Contributors",
						"Third party libs and tools", "Pre-fork contributors (TORCS)"):
			tabPane = winMgr.createWindow("CEGUIDemo/TabContentPane",
										  name + "/TapCredits%d" % tabInd)
			self.tacCred.addTab(tabPane)
			tabPane.setText(tabText)
			tabPane.setXPosition(PyCEGUI.UDim(0, 0))
			tabPane.setYPosition(PyCEGUI.UDim(0, 0))
			tabPane.setWidth(PyCEGUI.UDim(1, 0))
			tabPane.setHeight(PyCEGUI.UDim(1, 0))
			mclTable = winMgr.createWindow("CEGUIDemo/MultiColumnList",
										   name + "/MclTable%d" % tabInd)
			tabPane.addChildWindow(mclTable)
			mclTable.setXPosition(PyCEGUI.UDim(0, 0))
			mclTable.setYPosition(PyCEGUI.UDim(0, 0))
			mclTable.setWidth(PyCEGUI.UDim(1, 0))
			mclTable.setHeight(PyCEGUI.UDim(1, 0))
			mclTable.setProperty("Font", "TextSmall")
			mclTable.setProperty("ColumnsMovable", "False")
			mclTable.setProperty("ColumnsSizable", "True")
			mclTable.setProperty("SortSettingEnabled", "False")
			mclTable.addColumn("Name / Role", 0, PyCEGUI.UDim(0.35, 0))
			mclTable.addColumn("Main contributions / Contact", 1, PyCEGUI.UDim(0.65, 0))
			self.mclTables.append(mclTable)

			tabInd += 1

		# Retrieve window descendants created here.
		self.tacCred = window.getChild(name + "/TacCredits")
		self.btnBack = window.getChild(name + "/BtnBack")
		
		# Complete widget initialization.
		# TODO.
		# Note: Keep a reference to each listbox item,
		# otherwise they get garbage collected at the end of this function,
		# and then CEGUI crashes of course (see below : self.lbItems.append).
		self.lbItems = []
		for tabInd in range(len(self.mclTables)):
			for i in range(8):

				rowId = self.mclTables[tabInd].addRow()

				lbItem = PyCEGUI.ListboxTextItem("my name")
				self.mclTables[tabInd].setItem(lbItem, 0, rowId)
				self.lbItems.append(lbItem)

				lbItem = PyCEGUI.ListboxTextItem("my role")
				self.mclTables[tabInd].setItem(lbItem, 1, rowId)
				self.lbItems.append(lbItem)

				rowId = self.mclTables[tabInd].addRow()

				lbItem = PyCEGUI.ListboxTextItem("my contribs")
				self.mclTables[tabInd].setItem(lbItem, 0, rowId)
				self.lbItems.append(lbItem)

				lbItem = PyCEGUI.ListboxTextItem("my web site")
				self.mclTables[tabInd].setItem(lbItem, 1, rowId)
				self.lbItems.append(lbItem)

		return window

	# connectHandlers
	# - Wrapper method to define the subscription/listener relationships.
	# - If there are a lot, it may behoove the coder to encapsulate them in methods, then call those methods here.
	def connectHandlers(self):

		# Inherited connections.
		Menu.connectHandlers(self)

		# Specific connections.
		self.btnBack.subscribeEvent(PyCEGUI.PushButton.EventClicked, self, "onBackButtonClicked")

	def onBackButtonClicked(self, args):

		self.back()

	def onKeyDown(self, keyArgs):

		if keyArgs.scancode in (PyCEGUI.Key.Escape, PyCEGUI.Key.Return):
			self.onBackButtonClicked(keyArgs)
			return True

		return False
