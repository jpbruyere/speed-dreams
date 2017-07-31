#!/usr/bin/env python
# -*- coding: utf-8 -*-


"""Dialogs.

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
class DialogOKCancel(object):

	singleton = None

	def instance():
	
		if not DialogOKCancel.singleton:
			DialogOKCancel.singleton = DialogOKCancel()
			DialogOKCancel.singleton.initialize()
			DialogOKCancel.singleton.setup()
			
		return DialogOKCancel.singleton

	instance = staticmethod(instance)
	
	def __init__(self):

		pass
	
	# Initialize
	def initialize(self):
		
		self.window = PyCEGUI.WindowManager.getSingleton().loadWindowLayout("dialogokcancel.layout")

		# Retrieve the useful descendants.
		name = "DialogOKCancel/ImgBackground"
		frmBase = self.window.getChild(name)
		self.txtMessage = frmBase.getChild(name + "/TxtMessage")
		self.btnAccept = frmBase.getChild(name + "/BtnAccept")
		self.btnCancel = frmBase.getChild(name + "/BtnCancel")

		return self.window

	# - Wrapper method to define the subscription/listener relationships.
	# - If there are a lot, it may behoove the coder to encapsulate them in methods, then call those methods here.
	def connectHandlers(self):

		# Event subscriptions :
		# * keyboard.
		self.window.subscribeEvent(PyCEGUI.Window.EventKeyDown, self, 'onKeyDown')
			
		self.btnCancel.subscribeEvent(PyCEGUI.PushButton.EventClicked, self, "onCancelButtonClicked")
		self.btnAccept.subscribeEvent(PyCEGUI.PushButton.EventClicked, self, "onAcceptButtonClicked")

	# Setup
	def setup(self):

		self.connectHandlers()

	# Activate
	def show(self, rootWin, cbOnClosed, msg):

		self.rootWin = rootWin
		self.cbOnClosed = cbOnClosed
		self.rootWin.addChildWindow(self.window)
		self.txtMessage.setText(msg)
		self.window.show()
		self.window.activate() # Set focus (needed for EventKeyDown being received).
	
	def close(self, status=False):

		self.window.hide()
		self.window.deactivate()
		self.rootWin.removeChildWindow(self.window)
		self.cbOnClosed(status)

	def onCancelButtonClicked(self, args):

		print("OK / Cancel : Cancelled.")
		
		self.close(status=False)

	def onAcceptButtonClicked(self, args):

		print("OK / Cancel : Accepted.")
		
		self.close(status=True)

	def onKeyDown(self, keyArgs):

		if keyArgs.scancode == PyCEGUI.Key.Escape:
			keyArgs.handled = True
			self.onCancelButtonClicked(keyArgs)
			return True
		elif keyArgs.scancode == PyCEGUI.Key.Return:
			keyArgs.handled = True
			self.onAcceptButtonClicked(keyArgs)
			return True

		return False
