
import os
import glob
from xml.etree.ElementTree import ElementTree
from xml.etree.ElementTree import SubElement
from optparse import OptionParser

# pull in common functions
global options
import check_skins

try:
	import pysvn
	_has_pysvn = True
except ImportError:
	_has_pysvn = False

try:
	from git import *
	_has_pygit = True
except ImportError:
	_has_pygit = False

parser = OptionParser()

parser.set_defaults(cars=".", config=None, run=None, svn=None, git=None, proc=None, all=None)
parser.add_option("-c",  "--cars", dest="cars", help="cars directory")
parser.add_option("-C",  "--config", dest="config", help="path to 'speed-dreams-2' config directory")
parser.add_option("-r",  "--run", dest="run", help="command to run SpeedDreams")
parser.add_option("-p",  "--proc", dest="proc", help="command to process preview images")
parser.add_option("-a",  "--all", action="store_true", dest="all", help="process all previews regardless")

if _has_pysvn:
	parser.add_option("-s", "--svn", action="store_true", dest="svn", help="report svn version numbers")
if _has_pygit:
	parser.add_option("-g", "--git", action="store_true", dest="git", help="report git verison numbers")

(options, args) = parser.parse_args()

#---

def check_dir(args, dirname, names):

	car =  os.path.basename(dirname)

	for name in names:
		(root, ext) = os.path.splitext(name)
		if root == car and ext == ".xml":
			# Found config file
			print "checking", root
			print "---"

			path = os.sep.join([options.cars, car])
			model = ".".join([os.sep.join([path, car]), "acc"])

			check_skins.check_car(options, "human", "1", path, root, model)

#---

# check each of the cars in turn
os.path.walk(options.cars, check_dir, "")
