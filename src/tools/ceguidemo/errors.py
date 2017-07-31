#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# errors.py

"""Errors.

Currently defined errors include:

	InitializationError

"""

# Import: std
import sys


# InitializationError
class InitializationError(Exception):

	def __init__(self, msg):

		self.msg = msg

	def __str__(self):

		return ('InitializationError: %s' % self.msg)

