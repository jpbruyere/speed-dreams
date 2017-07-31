#!/usr/bin/env python
# -*- coding: utf-8 -*-


"""Track selection menu.


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


# Track selection menu
class MenuTrackSelect(MenuStandard):

	# Static data.
	Track = collections.namedtuple("Track", "name, description, authors, length, width, nPits, outline, preview")
	
	Tracks = \
	{
		"Grand Prix Circuits" : \
		{
			"Karwada" : Track(name="Karwada",
							  description="A well-known, fast and curvy circuit in Japan",
							  authors="Andrew Sumner, Echkard M. Jaeger",
							  length=6205, width=12, nPits=28,
							  outline="outline-karwada.png", preview="preview-karwada.jpg"),
			"Espie" : Track(name="Espie",
							description="An international racing circuit in France",
							authors="Eric Espie, Andrew Sumner, Echkard M. Jaeger",
							length=4441, width=13, nPits=20,
							outline="outline-espie.png", preview="preview-espie.jpg")
		},
		
		"Dirt" : \
		{
			"Garguree" : Track(name="Garguree",
							   description="A blast trough the australian country side with many hoops and a lot of dust",
							   authors="Andrew Sumner",
							   length=2206, width=10, nPits=16,
							   outline="outline-garguree.png", preview="preview-garguree.jpg"),
			"Mud Hell" : Track(name="Mud Hell",
							   description="An all-muddy track you'll probably never forget",
							   authors="Devel 666",
							   length=56220, width=8, nPits=16,
							   outline="outline-mudhell.png", preview="preview-mudhell.jpg")
		},
		
		"Road Tracks" : \
		{
			"Autodromo Lombaro" : Track(name="Autodromo Lombaro",
										description="An Italian race track with a 'bus stop'-like chicane",
										authors="Eric Espie, Bernhard Wymann, Echkard M. Jaeger",
										length=3244, width=15, nPits=20,
										outline="outline-lombaro.png", preview="preview-lombaro.jpg"),
			"Olethros Road" : Track(name="Olethros Road",
									description="A Greek narrow country road with dangerously bumpy sections",
									authors="Christos Dimitrikakis, Echkard M. Jaeger",
									length=6283, width=10, nPits=20,
									outline="outline-olethros.png", preview="preview-olethros.jpg")
		}
	}

	# Singleton pattern.
	singleton = None

	def instance():
	
		if not MenuTrackSelect.singleton:
			MenuTrackSelect.singleton = MenuTrackSelect()
			MenuTrackSelect.singleton.initialize()
			MenuTrackSelect.singleton.setup()
			
		return MenuTrackSelect.singleton

	instance = staticmethod(instance)

	def __init__(self):

		MenuStandard.__init__(self)

	# Initialize
	def initialize(self):

		name = "MenuTrackSelect"
		
		# No code written for this menu : use mandatory layout.
		window = MenuStandard.initialize(self, name=name, title="Select a track", layout="menutrackselect")

		# Retrieve the window descendants created here.
		self.btnBack  = window.getChild(name + "/BtnBack")
		self.btnNext  = window.getChild(name + "/BtnNext")
		self.cbxCat   = window.getChild(name + "/CbxCategory")
		self.cbxTrack = window.getChild(name + "/CbxTrack")
		self.txtDesc = window.getChild(name + "/TxtDesc")
		self.txtLength = window.getChild(name + "/TxtLength")
		self.txtWidth = window.getChild(name + "/TxtWidth")
		self.txtNPits = window.getChild(name + "/TxtNbPits")
		self.txtAuthors = window.getChild(name + "/TxtAuthors")
		self.imgOutline = window.getChild(name + "/ImgOutline")

		# Complete widget initialization.
		self.cbxCatItems = []
		for cat in self.Tracks:
			cbxItem = PyCEGUI.ListboxTextItem(cat)
			cbxItem.setSelectionBrushImage("CEGUIDemo", "ComboboxSelectionBrush")
			cbxItem.setSelectionColours(0xFF3FFFEE)
			self.cbxCat.addItem(cbxItem)
			self.cbxCatItems.append(cbxItem) # Avoid its being GC'd at return !
			
		self.cbxCatItems[0].setSelected(True) # Select first category.
		self.cbxCat.setText(self.cbxCatItems[0].getText())
		
		self.onCatChanged() # Initialise track combobox.

		# TODO.

		return window
		
	# connectHandlers
	# - Wrapper method to define the subscription/listener relationships.
	# - If there are a lot, it may behoove the coder to encapsulate them in methods, then call those methods here.
	def connectHandlers(self):

		# Inherited connections.
		MenuStandard.connectHandlers(self)

		# Specific connections.
		self.window.subscribeEvent(PyCEGUI.Window.EventSized, self, "onWindowSized")
		self.btnBack.subscribeEvent(PyCEGUI.PushButton.EventClicked, self, "onBackButtonClicked")
		self.btnNext.subscribeEvent(PyCEGUI.PushButton.EventClicked, self, "onNextButtonClicked")
		
		self.cbxCat.subscribeEvent(PyCEGUI.Combobox.EventListSelectionAccepted, self, "onCatChanged")
		self.cbxTrack.subscribeEvent(PyCEGUI.Combobox.EventListSelectionAccepted, self, "onTrackChanged")

	# Handler: buttonClicked
	def onNextButtonClicked(self, args):

		self.switchTo(MenuManager.get("CarSelect"))

	def onBackButtonClicked(self, args):

		self.back()

	def onCatChanged(self, notUsed=None):
		
		selCatName = self.cbxCat.getSelectedItem().getText()

		#print("onCatChanged(%s)" % selCatName)

		# Re-initialise track combo-box.
		self.cbxTrack.clearAllSelections()
		self.cbxTrack.resetList()
		self.cbxTrackItems = []
		for track in self.Tracks[selCatName].values():
			# auto_delete=False => items are not deleted when removed from the list by CEGUI ;
			# it's up to Python to take care of this, as it instanciates them !
			cbxItem = PyCEGUI.ListboxTextItem(track.name, auto_delete=False)
			cbxItem.setSelectionBrushImage("CEGUIDemo", "ComboboxSelectionBrush")
			cbxItem.setSelectionColours(0xFF3FFFEE)
			self.cbxTrack.addItem(cbxItem)
			self.cbxTrackItems.append(cbxItem) # Avoid its being GC'd at return !

		self.cbxTrackItems[0].setSelected(True)  # Select first track.
		self.cbxTrack.setText(self.cbxTrackItems[0].getText())

		self.onTrackChanged()

	def onTrackChanged(self, notUsed=None):
		
		selCatName = self.cbxCat.getSelectedItem().getText()
		selTrackName = self.cbxTrack.getSelectedItem().getText()

		print("onTrackChanged(cat=%s, track=%s)" % (selCatName, selTrackName))
		
		track = self.Tracks[selCatName][selTrackName]

		self.txtDesc.setText(track.description)
		self.txtLength.setText("%d m" % track.length)
		self.txtWidth.setText("%d m" % track.width)
		self.txtNPits.setText("%d m" % track.nPits)
		self.txtAuthors.setText(track.authors)

		# Update outline image.
		imgSetMgr = PyCEGUI.ImagesetManager.getSingleton()
		#if not imgSetMgr.???(track.outline):
		imgSetMgr.createFromImageFile(track.outline, track.outline)
		self.imgOutline.setProperty("Image", "set:%s image:full_image" % track.outline)

		# Update background image, respecting its aspect ratio through clipping.
		#if not imgSetMgr.???(track.preview):
		self.imgSetPreview = imgSetMgr.createFromImageFile(track.preview, track.preview)
		self.onWindowSized(PyCEGUI.WindowEventArgs(self.window))

	def onWindowSized(self, args):

		selCatName = self.cbxCat.getSelectedItem().getText()
		selTrackName = self.cbxTrack.getSelectedItem().getText()
		track = self.Tracks[selCatName][selTrackName]

		# Update background image clipping, according to new window size,
		# as we want to keep its aspect ratio.
		# Note the image assignment to the always existing "full_image" one
		# before undefining the target "undeformed" : prevents crashes :-)
		imgSize = self.imgSetPreview.getNativeResolution()
		clipX, clipY = self.getPreviewClipping(imgSize)
		self.window.setProperty("Image", "set:%s image:full_image" % track.preview)
		self.imgSetPreview.undefineImage("undeformed")
		self.imgSetPreview.defineImage("undeformed", PyCEGUI.Vector2(clipX, clipY),
									   PyCEGUI.Size(imgSize.d_width - 2*clipX, imgSize.d_height - 2*clipY),
									   PyCEGUI.Vector2(0,0))
		self.window.setProperty("Image", "set:%s image:undeformed" % track.preview)

	def getPreviewClipping(self, imgSize):

		winSize = self.window.getPixelSize()
		rFactor = (float(imgSize.d_width) / imgSize.d_height) \
				  / (float(winSize.d_width) / winSize.d_height)
		if rFactor >= 1:
			clipX = int(imgSize.d_width * (rFactor - 1.0) / 2.0)
			clipY = 0
		else:
			clipX = 0
			clipY = int(imgSize.d_height * (1.0 - rFactor) / 2.0)
		print("winSize=(%d,%d), imgSize=(%d,%d), r=%f" \
			  % (winSize.d_width, winSize.d_height, imgSize.d_width, imgSize.d_height, rFactor))
		print("clipRect=(%d,%d,%d,%d)" % (clipX, clipY, imgSize.d_width - clipX, imgSize.d_height - clipY))

		return clipX, clipY
