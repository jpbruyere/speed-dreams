#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# video.py


"""Video functionality.

This class brings together PyCEGUI and OpenGL into a comfortable interface.

"""

# Import: std
import sys
import time

# Import: OpenGL
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

# Import: PyCEGUI
import PyCEGUI
from PyCEGUIOpenGLRenderer import OpenGLRenderer

# Import: User
from constants import *
from errors import InitializationError

# Import: Configuration
from configuration import TheConfig


# Video
class Video(object):

	def __del__(self):
	
		OpenGLRenderer.destroySystem()
	
	# Initialize: OpenGL
	def initializeOpenGL(self):
		
		glutInit()
		glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA)
		glutInitWindowSize(1024, 640)
		glutInitWindowPosition(-1, -1)
		glutCreateWindow(NAME_ROOT_WINDOW)
		glutSetCursor(GLUT_CURSOR_NONE)

		# Handlers
		glutDisplayFunc(self.handlerDisplay)
		glutReshapeFunc(self.handlerReshape)

	# Initialize: PyCEGUI
	def initializePyCEGUI(self):
		
		if TheConfig.useConfigFile:
		
			# Needed for CEGUI config. file support
			self.renderer = OpenGLRenderer.create()
			PyCEGUI.System.create(self.renderer, None, None, None, None, "datafiles/cegui.config")
			
		else:
		
			# Simpler way, but no possible config. file
			self.renderer = OpenGLRenderer.bootstrapSystem()

	# Initialize
	def initialize(self):
		
		try:
			self.initializeOpenGL()
			self.initializePyCEGUI()
		except Exception, msg:
			raise InitializationError(msg)

	# Shutdown
	# - For implicit use, use the Python special method `__del__`.
	def shutdown(self):
		
		self.renderer.destroySystem()

	# Handler: Display
	# - This is called to refresh the screen.
	# - See PyOpenGL documentation.
	def handlerDisplay(self):

		# Inject time.
		thisTime = glutGet(GLUT_ELAPSED_TIME)
		elapsed = (thisTime - self.lastFrameTime) / 1000.0
		self.lastFrameTime = thisTime
		PyCEGUI.System.getSingleton().injectTimePulse(elapsed)

		# Render this frame
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
		PyCEGUI.System.getSingleton().renderGUI()
		glutPostRedisplay()
		glutSwapBuffers()
		time.sleep(0.01)

	# Handler: Reshape
	# - This is called when the window is resized and/or switches to fullscreen.
	# - See PyOpenGL documentation.
	def handlerReshape(self, width, height):

		glViewport(0, 0, width, height)
		glMatrixMode(GL_PROJECTION)
		glLoadIdentity()
		gluPerspective(60.0, width / height, 1.0, 50.0)
		glMatrixMode(GL_MODELVIEW)
		PyCEGUI.System.getSingleton().notifyDisplaySizeChanged(PyCEGUI.Size(width, height))

	# Main loop
	# - Set the initial values.
	# - This never returns; once this gets called, the application is driven entirely by events.
	def enterMainLoop(self):

		self.lastFrameTime = glutGet(GLUT_ELAPSED_TIME)
		glutMainLoop()
