#!/usr/bin/env python
# -*- coding: utf-8 -*-


"""Profiles menu.


"""

# Import: std
import sys

# Import: PyCEGUI
import PyCEGUI

# Import: Configuration
from configuration import TheConfig

# Import: Menu
from menu import Menu

# Profiles menu
class MenuProfiles(Menu):

	# Singleton pattern.
	singleton = None

	def instance():
	
		if not MenuProfiles.singleton:
			MenuProfiles.singleton = MenuProfiles()
			MenuProfiles.singleton.initialize()
			MenuProfiles.singleton.setup()
			
		return MenuProfiles.singleton

	instance = staticmethod(instance)

	def __init__(self):

		Menu.__init__(self)
	
	# Initialize
	def initialize(self):

		name = "MenuProfiles"

		# No code written for this menu : use mandatory layout.
		window = Menu.initialize(self, name=name, title="Profiles", layout="menuprofiles")
			
		# Retrieve window descendants created here.
		self.btnAccept = window.getChild(name + "/BtnAccept")
		self.btnCancel = window.getChild(name + "/BtnCancel")
		self.edxName = window.getChild(name + "/EdxName")
		self.cbxSkill = window.getChild(name + "/CbxSkill")
		self.lbxProfiles = window.getChild(name + "/LbxProfiles")
		self.btnAdd = window.getChild(name + "/BtnAdd")
		self.btnRemove = window.getChild(name + "/BtnRemove")
		
		# Complete widget initialization.
		self.cbxSkillItems = []
		for skill in ("Rookie", "Amateur", "Semi-pro", "Pro"):
			cbxItem = PyCEGUI.ListboxTextItem(skill)
			cbxItem.setSelectionBrushImage("CEGUIDemo", "ComboboxSelectionBrush")
			cbxItem.setSelectionColours(0xFF3FFFEE)
			self.cbxSkill.addItem(cbxItem)
			self.cbxSkillItems.append(cbxItem) # Avoid its being GC'd at return !
			if skill.startswith("Rookie"):
				self.cbxSkill.setText(cbxItem.getText())

		#self.lbProfItems = []
		self.lbItemId = 0
		for name in ("Mike", "Horst", "Paolo", "Albert", "Yuuki", "Bjorn",
					 "Jane", "Henri", "Francesca", "Joao", "Klaus"):
			# Obsolete Listbox use case.
			#lbItem = PyCEGUI.ListboxTextItem(name, auto_delete=False)
			#lbItem.setSelectionBrushImage("CEGUIDemo", "ListboxSelectionBrush")
			#lbItem.setSelectionColours(0xFF3FFFEE)
			#lbItem.setSelectionBrushImage("CEGUIDemo", "ListboxSelectionBrush")
			#lbItem.setSelectionColours(0xFF3FFFEE)
			#self.lbProfItems.append(lbItem) # Avoid its being GC'd at return !

			lbItem = PyCEGUI.WindowManager.getSingleton().createWindow("CEGUIDemo/ListboxItem", "profile_%d" % self.lbItemId)
			self.lbItemId += 1
			lbItem.setText(name)
			self.lbxProfiles.addItem(lbItem)

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
		self.btnAdd.subscribeEvent(PyCEGUI.PushButton.EventClicked, self, "onAddButtonClicked")
		self.btnRemove.subscribeEvent(PyCEGUI.PushButton.EventClicked, self, "onRemoveButtonClicked")
		self.edxName.subscribeEvent(PyCEGUI.Editbox.EventTextAccepted, self, "onNameAccepted")
		self.lbxProfiles.subscribeEvent(PyCEGUI.ItemListbox.EventSelectionChanged, self, "onListSelectionChanged")

	def onCancelButtonClicked(self, args):

		print("Profiles : Cancelled.")
		self.back()

	def onAcceptButtonClicked(self, args):

		print("Profiles : Accepted.")
		self.back()

	def onListSelectionChanged(self, notUsed):

		lbItem = self.lbxProfiles.getFirstSelectedItem()
		if lbItem:
			self.edxName.setText(lbItem.getText())
			self.btnRemove.enable()
		else:
			self.edxName.setText("")
			self.btnRemove.disable()

	def onNameAccepted(self, notUsed):

		lbItem = self.lbxProfiles.getFirstSelectedItem()
		if lbItem:
			lbItem.setText(self.edxName.getText())
			# Obsolete Listbox use case.
			#self.lbxProfiles.handleUpdatedItemData()
			#self.lbxProfiles.ensureItemIsVisible(lbItem)
			self.lbxProfiles.ensureItemIsVisibleVert(lbItem)

	def onAddButtonClicked(self, args):

		self.lbxProfiles.clearAllSelections()
		
		# Obsolete Listbox use case.
		#lbItem = PyCEGUI.ListboxTextItem("-- no one --", auto_delete=False)
		#lbItem.setSelectionBrushImage("CEGUIDemo", "ListboxSelectionBrush")
		#lbItem.setSelectionColours(0xFF3FFFEE)
		#self.lbProfItems.append(lbItem) # Avoid its being GC'd at return !
		
		lbItem = PyCEGUI.WindowManager.getSingleton().createWindow("CEGUIDemo/ListboxItem", "profile_%d" % self.lbItemId)
		self.lbItemId += 1
		lbItem.setText("-- no one --")

		self.lbxProfiles.addItem(lbItem)
		
		# Obsolete Listbox use case.
		#self.lbxProfiles.setItemSelectState(lbItem, True)
		#self.lbxProfiles.ensureItemIsVisible(lbItem)
		lbItem.setSelected(True)
		self.lbxProfiles.ensureItemIsVisibleVert(lbItem)


	def onRemoveButtonClicked(self, args):

		lbItem = self.lbxProfiles.getFirstSelectedItem()
		while lbItem:
			#print("removing %s (%s)" % (lbItem, lbItem.getText()))
			# Obsolete Listbox use case.
			#lbItem2Rremove = lbItem
			#lbItem = self.lbxProfiles.getNextSelected(lbItem)
			#self.lbxProfiles.setItemSelectState(lbItem2Rremove, False)
			#self.lbProfItems.remove(lbItem2Rremove)
			lbItem.setSelected(False)
			self.lbxProfiles.removeItem(lbItem)
			lbItem = self.lbxProfiles.getNextSelectedItem()

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
