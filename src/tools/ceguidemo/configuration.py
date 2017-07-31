#!/usr/bin/env python
# -*- coding: utf-8 -*-


"""Configuration.

Singleton for app. configuration management

"""

# Configuration
class _Configuration(object):

	# Ctor : default settings.
	def __init__(self):

		# If True, load menus from .layout files (otherwise, run dedicated code).
		self.useLayouts = False

		# If True, initialize CEGUI from the configuration file in datafiles/cegui.config
		# (otherwise, initialize it from the code).
		self.useConfigFile = False

# TheConfig : the singleton.
TheConfig = _Configuration()
