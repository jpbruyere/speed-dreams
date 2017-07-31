#!/usr/bin/env python
# -*- coding: utf-8 -*-


"""Loading menu.


"""

# Import: std
import sys
import time
import collections

# Import: PyCEGUI
import PyCEGUI

# Import: Configuration
from configuration import TheConfig

# Import: Menu
from menu import Menu
from menumanager import MenuManager


# Loading menu
class MenuLoading(Menu):

	# Static data.
	Msg = collections.namedtuple("Msg", "time, dur, txt")

	Messages = \
		(Msg(time= 0.0, dur=0.1, txt="Loading Forza track by A. Sumner (5784m long, 11m wide)"),
		 Msg(time= 0.1, dur=0.9, txt="Determining starting order"),
		 Msg(time= 1.0, dur=1.0, txt="Preparing starting grid"),
		 Msg(time= 2.0, dur=2.0, txt="Loading physics engine (Simu V2.1)"),
		 Msg(time= 4.0, dur=1.0, txt="Loading Aarne Fisher driver (Boxer 96)"),
		 Msg(time= 5.0, dur=1.0, txt="Loading Jacky Graham driver (Spirit 300)"),
		 Msg(time= 6.0, dur=1.0, txt="Loading Tony Davis driver (Cavallo 360)"),
		 Msg(time= 7.0, dur=2.0, txt="Loading graphics for Forza track"),
		 Msg(time= 9.0, dur=1.0, txt="Running pre-start"),
		 Msg(time=10.0, dur=2.0, txt="Loading graphics for all cars"),
		 Msg(time=12.0, dur=1.0, txt="Loading sounds for all cars"),
		 Msg(time=13.0, dur=1.0, txt="Ready"),
		 Msg(time=14.0, dur=1.0, txt="Set"),
		 Msg(time=15.0, dur=1.0, txt="Go"),
		 Msg(time=16.0, dur=0.0, txt="Now, you should be racing"))
	
	# Singleton pattern.
	singleton = None

	def instance():
	
		if not MenuLoading.singleton:
			MenuLoading.singleton = MenuLoading()
			MenuLoading.singleton.initialize()
			MenuLoading.singleton.setup()
			
		return MenuLoading.singleton

	instance = staticmethod(instance)

	def __init__(self):

		Menu.__init__(self)
	
	# Initialize
	def initialize(self):

		name = "MenuLoading"

		# No code written for this menu : use mandatory layout.
		window = Menu.initialize(self, name=name, title="Loading", layout="menuloading")
			
		# Retrieve window descendants created here.
		self.prbProgress = window.getChild(name + "/PrbProgress")
		self.scpMessages = window.getChild(name + "/ScpMessages")
		
		# Complete widget initialization.

		return window

	# connectHandlers
	# - Wrapper method to define the subscription/listener relationships.
	# - If there are a lot, it may behoove the coder to encapsulate them in methods, then call those methods here.
	def connectHandlers(self):

		# Inherited connections.
		Menu.connectHandlers(self)

		# Specific connections.
		# None (onUpdate inherited).

	def onActivated(self, args):
			
		print("%s.onActivated" % self.__class__.__name__)

		sts = Menu.onActivated(self, args)

		self.actTime = self.currTime
		self.curInd = -1

		if self.scpMessages.getContentPane().getChildCount() > 0:
			for ind in range(len(self.Messages)):
				txtChld = self.scpMessages.getContentPane().getChild("MenuLoading/ScpMessages/Msg%d" % ind)
				self.scpMessages.getContentPane().removeChildWindow(txtChld)
				txtChld.destroy()
		
		self.prbProgress.setProgress(0)

		return sts
		
	def onUpdate(self, args):

		Menu.onUpdate(self, args)

		sinceAct = (self.currTime - self.actTime) / 1000.0
		
		#print("Since=%.2f, curr=%d" % (sinceAct, self.curInd))

		if self.curInd < 0 \
		   or (sinceAct > self.Messages[self.curInd].time + self.Messages[self.curInd].dur):

			# Next message : if none left, wait a little, and then switch to next menu. 
			if self.curInd == len(self.Messages) - 1:

				if sinceAct > self.Messages[self.curInd].time + self.Messages[self.curInd].dur + 2.0:
					self.switchTo(MenuManager.get("Results"))
				else:
					self.prbProgress.hide()
					time.sleep(0.01) # Let the CPU keep cool (no hurry).

			# Next message : if at least one left ... 
			else:

				self.curInd = self.curInd + 1
				
				# Create and add the static text for the new message.
				scpItem = PyCEGUI.WindowManager.getSingleton().createWindow("CEGUIDemo/StaticText", "MenuLoading/ScpMessages/Msg%d" % self.curInd)
				scpItem.setText(self.Messages[self.curInd].txt + " ...")
				scpItem.setProperty("FrameEnabled", "false")
				scpItem.setProperty("BackgroundEnabled", "false")
				scpItem.setXPosition(PyCEGUI.UDim(0.0, 0.0))
				scpItem.setYPosition(PyCEGUI.UDim(1.0 + self.curInd * 0.10, 0.0))
				scpItem.setWidth(PyCEGUI.UDim(1.0, 0.0))
				scpItem.setHeight(PyCEGUI.UDim(0.10, 0.0))
				self.scpMessages.addChildWindow(scpItem)

				# Adjust alpha for all messages.
				nChildren = self.scpMessages.getContentPane().getChildCount()
				for chldInd in range(nChildren):
					txtChld = self.scpMessages.getContentPane().getChildAtIdx(chldInd)
					txtChld.setAlpha(0.85 ** (nChildren - 1 - chldInd))
				
				# Scroll down to show the added static text.
				self.scpMessages.setVerticalScrollPosition(1.0)

				# Hide any scrollbar (might get shown after every call to addChildWindow).
				self.scpMessages.getVertScrollbar().hide()
				self.scpMessages.getHorzScrollbar().hide()

				# Reset progress-bar.
				self.prbProgress.setProgress(0)
				self.prbProgress.show()

		else:

			# Update progress-bar.
			progress = (sinceAct - self.Messages[self.curInd].time) / self.Messages[self.curInd].dur
			self.prbProgress.setProgress(progress)
