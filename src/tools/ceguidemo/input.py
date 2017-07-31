#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# input.py


"""Input functionality.

This class sets up input processing by creating handlers for GLUT that inject
input into PyCEGUI.

GLUT to CEGUI keyboard mapping from Leftium's PyCEGUI port of Falagard demo console
(http://www.cegui.org.uk/phpBB2/viewtopic.php?f=4&t=5425)

"""

# Import: std
import sys
import string

# Import: OpenGL
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

# Import: PyCEGUI
import PyCEGUI

# Import: User
from constants import *
from errors import InitializationError


# Input
class Input(object):

	# GLUT to CEGUI keyboard mapping.
	mapping = dict([(ord(c), getattr(PyCEGUI.Key, c.upper()))
				   for c in string.ascii_letters])
	mapping.update({
				 96: PyCEGUI.Key.Grave,        126: PyCEGUI.Key.Grave,
				 49: PyCEGUI.Key.One,           33: PyCEGUI.Key.One,
				 50: PyCEGUI.Key.Two,           64: PyCEGUI.Key.At,
				 51: PyCEGUI.Key.Three,         35: PyCEGUI.Key.Three,
				 52: PyCEGUI.Key.Four,          36: PyCEGUI.Key.Four,
				 53: PyCEGUI.Key.Five,          37: PyCEGUI.Key.Five,
				 54: PyCEGUI.Key.Six,           94: PyCEGUI.Key.Six,
				 55: PyCEGUI.Key.Seven,         38: PyCEGUI.Key.Seven,
				 56: PyCEGUI.Key.Eight,         42: PyCEGUI.Key.Multiply,
				 57: PyCEGUI.Key.Nine,          40: PyCEGUI.Key.Nine,
				 48: PyCEGUI.Key.Zero,          41: PyCEGUI.Key.Zero,
				 45: PyCEGUI.Key.Minus,         95: PyCEGUI.Key.Underline,
				 61: PyCEGUI.Key.Equals,        43: PyCEGUI.Key.Equals,
				 91: PyCEGUI.Key.LeftBracket,  123: PyCEGUI.Key.LeftBracket,
				 93: PyCEGUI.Key.RightBracket, 125: PyCEGUI.Key.RightBracket,
				 59: PyCEGUI.Key.Semicolon,     58: PyCEGUI.Key.Colon,
				 39: PyCEGUI.Key.Apostrophe,    34: PyCEGUI.Key.Apostrophe,
				 92: PyCEGUI.Key.Backslash,    124: PyCEGUI.Key.Backslash,
				 44: PyCEGUI.Key.Comma,         60: PyCEGUI.Key.Comma,
				 46: PyCEGUI.Key.Period,        62: PyCEGUI.Key.Period,
				 47: PyCEGUI.Key.Slash,         63: PyCEGUI.Key.Slash,
				 13: PyCEGUI.Key.Return,
				  8: PyCEGUI.Key.Backspace,
				  9: PyCEGUI.Key.Tab,
				 32: PyCEGUI.Key.Space,
				127: PyCEGUI.Key.Delete,
				 27: PyCEGUI.Key.Escape})

	specialKeyMap = {
			GLUT_KEY_F1: PyCEGUI.Key.F1,
			GLUT_KEY_F2: PyCEGUI.Key.F2,
			GLUT_KEY_F3: PyCEGUI.Key.F3,
			GLUT_KEY_F4: PyCEGUI.Key.F4,
			GLUT_KEY_F5: PyCEGUI.Key.F5,
			GLUT_KEY_F6: PyCEGUI.Key.F6,
			GLUT_KEY_F7: PyCEGUI.Key.F7,
			GLUT_KEY_F8: PyCEGUI.Key.F8,
			GLUT_KEY_F9: PyCEGUI.Key.F9,
			GLUT_KEY_F10: PyCEGUI.Key.F10,
			GLUT_KEY_F11: PyCEGUI.Key.F11,
			GLUT_KEY_F12: PyCEGUI.Key.F12,
			GLUT_KEY_LEFT: PyCEGUI.Key.ArrowLeft,
			GLUT_KEY_UP: PyCEGUI.Key.ArrowUp,
			GLUT_KEY_RIGHT: PyCEGUI.Key.ArrowRight,
			GLUT_KEY_DOWN: PyCEGUI.Key.ArrowDown,
			GLUT_KEY_PAGE_UP: PyCEGUI.Key.PageUp,
			GLUT_KEY_PAGE_DOWN: PyCEGUI.Key.PageDown,
			GLUT_KEY_HOME: PyCEGUI.Key.Home,
			GLUT_KEY_END: PyCEGUI.Key.End,
			GLUT_KEY_INSERT: PyCEGUI.Key.Insert }

	def ascii2Scancode(self, a):
	
		a = ord(a)
		return self.mapping[a] if (a in self.mapping) else 0

	def special2Scancode(self, c):
	
		return self.specialKeyMap[c] if (c in self.specialKeyMap) else 0

	# Constructor.
	def __init__(self):
	
		self.glut_modifiers = \
			dict(shift = dict(is_held=False,
							  bit_flag=GLUT_ACTIVE_SHIFT,
							  scancode=PyCEGUI.Key.LeftShift),
				 ctrl = dict(is_held=False,
							 bit_flag=GLUT_ACTIVE_CTRL,
							 scancode=PyCEGUI.Key.LeftControl),
				 alt = dict(is_held=False,
							bit_flag=GLUT_ACTIVE_ALT,
							scancode=PyCEGUI.Key.LeftAlt))

	# Initialize: Handlers
	def initializeHandlers(self):
	
		glutKeyboardFunc(self.handlerNormalKeyDown)
		glutSpecialFunc(self.handlerSpecialKeyDown)

		glutKeyboardUpFunc(self.handlerNormalKeyUp)
		glutSpecialUpFunc(self.handlerSpecialKeyUp)

		# The difference between these two is that the passive one is called when there is
		# mouse motion while no buttons are pressed, and the other is called when there
		# is mouse motion while buttons are pressed. See PyOpenGL documentation.
		glutMotionFunc(self.handlerMouseMotion)
		glutPassiveMotionFunc(self.handlerMouseMotion)

		glutMouseFunc(self.handlerMouseButton)

	# Initialize
	def initialize(self):
	
		try:
			self.initializeHandlers()
		except Exception, msg:
			raise InitializationError(msg)

	def handleModifierKeys(self):

		status = glutGetModifiers()

		for name, key in self.glut_modifiers.items():
			if (status & key['bit_flag']):
				if not key['is_held']:
					key['is_held'] = True
					PyCEGUI.System.getSingleton().injectKeyDown(key['scancode'])
					print("handleModifierKeys: Down %s" % name)
			elif key['is_held']:
				key['is_held'] = False
				PyCEGUI.System.getSingleton().injectKeyUp(key['scancode'])
				print("handleModifierKeys: Up %s " % name)

	# Handler: Normal Key Down
	def handlerNormalKeyDown(self, key, x, y):

		self.handleModifierKeys()

		key = key.encode('ascii', 'ignore')
		scancode = self.ascii2Scancode(key)
		if scancode:
			PyCEGUI.System.getSingleton().injectKeyDown(int(scancode))
		PyCEGUI.System.getSingleton().injectChar(ord(key))
		
		return False

	# Handler: Normal Key Up
	def handlerNormalKeyUp(self, key, x, y):

		self.handleModifierKeys()

		key = key.encode('ascii', 'ignore')
		scancode = self.ascii2Scancode(key)
		if scancode:
			PyCEGUI.System.getSingleton().injectKeyDown(int(scancode))
		PyCEGUI.System.getSingleton().injectKeyUp(int(scancode))
		
		return False

	# Handler: Special Key Down
	def handlerSpecialKeyDown(self, key, x, y):

		self.handleModifierKeys()

		scancode = self.special2Scancode(key)
		if scancode:
			PyCEGUI.System.getSingleton().injectKeyDown(int(scancode))

		return False

	# Handler: Special Key Up
	def handlerSpecialKeyUp(self, key, x, y):

		self.handleModifierKeys()

		scancode = self.special2Scancode(key)
		if scancode:
			PyCEGUI.System.getSingleton().injectKeyUp(int(scancode))

		return False

	# Handler: Mouse Button
	def handlerMouseButton(self, button, state, x, y):

		self.handleModifierKeys()

		if button == GLUT_LEFT_BUTTON:
			if state == GLUT_UP:
				PyCEGUI.System.getSingleton().injectMouseButtonUp(PyCEGUI.LeftButton)
			else:
				PyCEGUI.System.getSingleton().injectMouseButtonDown(PyCEGUI.LeftButton)

		# A thought is to turn this into an `else` clause; however, this implies that any
		# button besides the left is interpreted as the right button - this seems undesirable
		# for any mouse with more than two buttons.
		elif button == GLUT_RIGHT_BUTTON:
			if state == GLUT_UP:
				PyCEGUI.System.getSingleton().injectMouseButtonUp(PyCEGUI.RightButton)
			else:
				PyCEGUI.System.getSingleton().injectMouseButtonDown(PyCEGUI.RightButton)

		elif button == GLUT_MIDDLE_BUTTON:
			if state == GLUT_UP:
				PyCEGUI.System.getSingleton().injectMouseButtonUp(PyCEGUI.MiddleButton)
			else:
				PyCEGUI.System.getSingleton().injectMouseButtonDown(PyCEGUI.MiddleButton)

		# Do what we can about the poor GLUT mouse wheel support ...
		elif button in (3, 4):
			if state != GLUT_UP: # Ignore redundant UP events.
				PyCEGUI.System.getSingleton().injectMouseWheelChange((3.5 - button) / 2)
		
		return False

	# Handler: Mouse Motion
	# - This might seem arbitrary, but in fact this is required or else the position of the mouse
	# will never be updated inside the window.
	def handlerMouseMotion(self, x, y):

		PyCEGUI.System.getSingleton().injectMousePosition(x, y)
		
		return False
