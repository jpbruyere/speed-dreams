#!/usr/bin/env python
# -*- coding: utf-8 -*-


"""Menu factory.


"""

# Import: Menus manager
from menumanager import MenuManager

# Import: Menus
from menucredits import MenuCredits
from menuoptions import MenuOptions
from menuprofiles import MenuProfiles
from menumain import MenuMain
from menutrackselect import MenuTrackSelect
from menucarselect import MenuCarSelect
from menuloading import MenuLoading
from menuresults import MenuResults


class MenuFactory:

	def initialize():

		MenuManager.register("Credits",  MenuCredits.instance)
		MenuManager.register("Options",  MenuOptions.instance)
		MenuManager.register("Profiles", MenuProfiles.instance)

		MenuManager.register("Main",        MenuMain.instance)
		MenuManager.register("TrackSelect", MenuTrackSelect.instance)
		MenuManager.register("CarSelect",   MenuCarSelect.instance)
		MenuManager.register("Loading",     MenuLoading.instance)
		MenuManager.register("Results",     MenuResults.instance)

	initialize = staticmethod(initialize)
