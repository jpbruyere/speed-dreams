
import os
from xml.etree.ElementTree import ElementTree

import check_skins

from optparse import OptionParser

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

try:
	from PIL import Image, ImageFile
	_has_PIL = True
	ImageFile.MAXBLOCK=2**20
except ImportError:
	_has_PIL = False

parser = OptionParser()

parser.set_defaults(dir=".", cars=".", config=None, run=None, svn=None, git=None, proc=None, all=None)
parser.add_option("-d",  "--dir", dest="dir", help="driver directory")
parser.add_option("-c",  "--cars", dest="cars", help="cars directory")
parser.add_option("-C",  "--config", dest="config", help="path to '.speed-dreams' config directory")
parser.add_option("-r",  "--run", dest="run", help="command to run SpeedDreams")
parser.add_option("-p",  "--proc", dest="proc", help="command to process preview images")
parser.add_option("-a",  "--all", action="store_true", dest="all", help="process all previews regardless")

if _has_pysvn:
	parser.add_option("-s", "--svn", action="store_true", dest="svn", help="report svn version numbers")
if _has_pygit:
	parser.add_option("-g", "--git", action="store_true", dest="git", help="report git verison numbers")

(options, args) = parser.parse_args()


print "Checking", args[0]
print "---"

tree = ElementTree().parse(os.sep.join([options.dir, args[0]]))

p = tree.find("section/section")

for item in list(p):
	index = item.attrib["name"]

	for driver in list(item):
		if (driver.attrib["name"] == "name"):
			name = driver.attrib["val"]
		
		if (driver.attrib["name"] == "car name"):
			car = driver.attrib["val"]
		
	print index, ":", name, "(", car, ")"

	module = os.path.splitext(args[0])[0]

	# Check acc model
	model = ".".join([os.sep.join([options.cars, car, car]), "acc"])
	path = os.sep.join([options.dir, index])

	check_skins.check_car(options, module, index, path, car, model)

