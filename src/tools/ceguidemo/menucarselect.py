#!/usr/bin/env python
# -*- coding: utf-8 -*-


"""Car selection menu.


"""

# Import: std
import sys
import collections

# Import: PyCEGUI
import PyCEGUI

# Import: Configuration
from configuration import TheConfig

# Import: Menu
from menustandard import MenuStandard 
from menumanager import MenuManager


# Car selection menu
class MenuCarSelect(MenuStandard):

	# Static cars data.
	Car = collections.namedtuple("Car", "name, topSpeed, acceleration, cornering, skins")

	Cars = \
	{
		"Supercars" : \
		{
			"Lynx 220" : Car(name="Lynx 220",
							 topSpeed=0.80, acceleration=0.15, cornering=0.10,
							 skins={ "standard" : "sc-lynx-220-preview.jpg",
									 "all-tune" : "sc-lynx-220-all-tune-preview.jpg",
									 "red-lord" : "sc-lynx-220-red-lord-preview.jpg"}),
			"Murasama NSX" : Car(name="Murasama NSX",
								 topSpeed=0.60, acceleration=0.15, cornering=0.20,
								 skins={ "standard" : "sc-murasama-nsx-preview.jpg", 
										 "amami" : "sc-murasama-nsx-amami-preview.jpg", 
										 "tribal" : "sc-murasama-nsx-tribal-preview.jpg"})
		},
		
		"LS-GT1" : \
		{
			"Vulture V6R" : Car(name="Vulture V6R",
								topSpeed=0.85, acceleration=0.25, cornering=0.15,
								skins={ "standard" : "ls1-vulture-v6r-preview.jpg",
										"beast" : "ls1-vulture-v6r-beast-preview.jpg",
										"jd1" : "ls1-vulture-v6r-jd1-preview.jpg"}),
			"Zentek Z7R" : Car(name="Zentek Z7R",
							   topSpeed=0.80, acceleration=0.20, cornering=0.10,
							   skins={ "standard" : "ls1-zentek-z7r-preview.jpg",
									   "moemoe" : "ls1-zentek-z7r-moemoe-preview.jpg",
									   "telemachi" : "ls1-zentek-z7r-telemachi-preview.jpg"})
		}
	}

	# Singleton pattern.
	singleton = None

	def instance():
	
		if not MenuCarSelect.singleton:
			MenuCarSelect.singleton = MenuCarSelect()
			MenuCarSelect.singleton.initialize()
			MenuCarSelect.singleton.setup()
			
		return MenuCarSelect.singleton

	instance = staticmethod(instance)

	def __init__(self):

		MenuStandard.__init__(self)
	
	# Initialize
	def initialize(self):

		name = "MenuCarSelect"

		# No code written for this menu : use mandatory layout.
		window = MenuStandard.initialize(self, name=name, title="Select a car", layout="menucarselect")

		# Retrieve the window descendants created here.
		self.btnBack = window.getChild(name + "/BtnBack")
		self.btnStart = window.getChild(name + "/BtnStart")
		self.prbTopSpd = window.getChild(name + "/PrbTopSpeed")
		self.prbAccel = window.getChild(name + "/PrbAcceleration")
		self.prbCorner = window.getChild(name + "/PrbCornering")
		self.treCars = window.getChild(name + "/TreCars")
		self.imgPreview = window.getChild(name + "/ImgCarPreview")

		# Complete widget initialization.
		self.treCars.initialise()
		self.lstTreeItems = []
		self.dictTreeItemID2Car = {}
		
		treeItemId = 0
		rootTreeItem = PyCEGUI.TreeItem("Cars", item_id=treeItemId)
		treeItemId += 1
		rootTreeItem.setSelectionBrushImage("CEGUIDemo", "TreeSelectionBrush")
		rootTreeItem.setFont("MenuNormal")
		rootTreeItem.toggleIsOpen()
		self.treCars.addItem(rootTreeItem)
		self.lstTreeItems.append(rootTreeItem) # Avoid its being GC'd at return !

		for cat in self.Cars:
			catTreeItem = PyCEGUI.TreeItem(cat, item_id=treeItemId)
			treeItemId += 1
			catTreeItem.setSelectionBrushImage("CEGUIDemo", "TreeSelectionBrush")
			catTreeItem.setFont("MenuNormal")
			catTreeItem.toggleIsOpen()
			rootTreeItem.addItem(catTreeItem)
			self.lstTreeItems.append(catTreeItem) # Avoid its being GC'd at return !
			for car in self.Cars[cat].values():
				carTreeItem = PyCEGUI.TreeItem(car.name, item_id=treeItemId)
				treeItemId += 1
				carTreeItem.setSelectionBrushImage("CEGUIDemo", "TreeSelectionBrush")
				carTreeItem.setFont("MenuNormal")
				carTreeItem.toggleIsOpen()
				catTreeItem.addItem(carTreeItem)
				self.lstTreeItems.append(carTreeItem) # Avoid its being GC'd at return !
				self.dictTreeItemID2Car[carTreeItem.getID()] = car
				for name, file in car.skins.items():
					skinTreeItem = PyCEGUI.TreeItem(name, item_id=treeItemId)
					treeItemId += 1
					skinTreeItem.setSelectionBrushImage("CEGUIDemo", "TreeSelectionBrush")
					skinTreeItem.setFont("MenuMedium")
					carTreeItem.addItem(skinTreeItem)
					self.lstTreeItems.append(skinTreeItem) # Avoid its being GC'd at return !
					self.dictTreeItemID2Car[skinTreeItem.getID()] = car

		return window
		
	# connectHandlers
	# - Wrapper method to define the subscription/listener relationships.
	# - If there are a lot, it may behoove the coder to encapsulate them in methods, then call those methods here.
	def connectHandlers(self):

		# Inherited connections.
		MenuStandard.connectHandlers(self)

		# Specific connections.
		self.btnBack.subscribeEvent(PyCEGUI.PushButton.EventClicked, self, "onBackButtonClicked")
		self.btnStart.subscribeEvent(PyCEGUI.PushButton.EventClicked, self, "onStartButtonClicked")
		self.treCars.subscribeEvent(PyCEGUI.Tree.EventSelectionChanged, self, "onTreeSelectionChanged")

	# Handler: buttonClicked
	def onStartButtonClicked(self, args):

		self.switchTo(MenuManager.get("Loading"))

	def onBackButtonClicked(self, args):

		self.back()
		
	def onTreeSelectionChanged(self, args):
	
		treeItem = args.treeItem
		if treeItem and treeItem.getID() in self.dictTreeItemID2Car:
			car = self.dictTreeItemID2Car[treeItem.getID()]
			self.prbTopSpd.setProgress(car.topSpeed)
			self.prbCorner.setProgress(car.cornering)
			self.prbAccel.setProgress(car.acceleration)
			if treeItem.getText() in car.skins:
				skinFile = car.skins[treeItem.getText()]
			else:
				skinFile = car.skins["standard"]
			imgSetMgr = PyCEGUI.ImagesetManager.getSingleton()
			imgSetMgr.createFromImageFile(skinFile, skinFile)
			self.imgPreview.setProperty("Image", "set:%s image:full_image" % skinFile)
			self.imgPreview.show()
		else:
			self.prbTopSpd.setProgress(0)
			self.prbCorner.setProgress(0)
			self.prbAccel.setProgress(0)
			self.imgPreview.hide()
