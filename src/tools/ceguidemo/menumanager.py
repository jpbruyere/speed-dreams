#!/usr/bin/env python
# -*- coding: utf-8 -*-


"""Menu manager.

A dictionary of menu instance() functions (see MenuFactory)
"""

class MenuManager:
	
	menus = { }

	def register(menuName, menuFactory):
	
		MenuManager.menus[menuName] = menuFactory

	register = staticmethod(register)

	def get(menuName):
	
		return MenuManager.menus[menuName]()
	
	get = staticmethod(get)
