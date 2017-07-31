
import os
import glob
from xml.etree.ElementTree import ElementTree
from xml.etree.ElementTree import SubElement

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


def check_version(myfile):
	# Check myfile exists
	if not os.access(myfile, os.R_OK):
		return None

	# Return SVN revision
	if _has_pysvn and options.svn:
		client = pysvn.Client()
		entry = client.info(myfile)
		return entry.commit_revision.number

	# Return GIT revision
	if _has_pygit and options.git:
		repo = Repo(myfile)
		headcommit = repo.head.commit
		if headcommit:
			for line in headcommit.message.splitlines():
				if line.startswith("git-svn-id:"):
					return int(line.split("@", 1)[1].split(" ",1)[0])
		else:
			return -1

	# Fall through when SVN/GIT not present
	return 1

#---

def get_screenshot(module, index, car, skin, skintargets):
	if not options.config or not options.run:
		return None

	skin_done = False

	config_file = os.sep.join([options.config, "config/raceman/practice.xml"])

	if not os.access(config_file, os.R_OK):
		print "Can't find 'practice.xml' (", config_file, ","
		return None

	my_ele = ElementTree()
	p = my_ele.parse(config_file)
	q = p.findall("section")

	for i in list(q):
		if i.attrib["name"] == "Drivers" or i.attrib["name"] == "Drivers Start List":
			for k in list(i):
				if k.attrib["name"] == "focused module":
					k.set("val", module)
				if k.attrib["name"] == "focused idx":
					k.set("val", index)

			j = i.find("section")
	
			# modify attributes
			if j is not None:
				for k in list(j):
					if k.attrib["name"] == "idx":
						k.set("val", index)
					if k.attrib["name"] == "module":
						k.set("val", module)
					if k.attrib["name"] == "skin targets":
						k.set("val", skintargets)
					if k.attrib["name"] == "skin name":
						skin_done = True
						if skin:
							k.set("val", skin)
						else:
							j.remove(k)

			if not skin_done and skin:
				# Need to add skin attribute
				SubElement(j, "attstr", {'name':"skin name", 'val':skin})

			'''
			# dump attributes
			for k in list(j):
				print ":", k.attrib["name"], k.attrib["val"]
			'''

		if i.attrib["name"] == "Driver Info":
			j = i.find("section/section/section")

			for k in list(j):
				if k.attrib["name"] == "car name":
					k.set("val", car)

	# Store the changes
	my_ele.write(config_file)

	# Run the game
	os.system(options.run)

	# return the filename of screen shot

#---

def process_screenshot(preview):
	screenshot_files = os.listdir(os.sep.join([options.config,"screenshots"]))
	if screenshot_files:
		screenshot_file = screenshot_files[0]

		screenshot = Image.open(os.sep.join([options.config, "screenshots", screenshot_file]))
		scaled = screenshot.resize((800,500), Image.ANTIALIAS)

		scaled.save(preview, quality=95, optimize=True, subsampling='4:4:4')
		os.remove(os.sep.join([options.config, "screenshots", screenshot_file]))

#---

def check_car(myoptions, module, index, path, car, model):
	global options
	options = myoptions

	# Check acc model
	model_ver = check_version(model)

	if (model_ver == None):
		print "Warning: ACC model not in directory"

	# Checking for standard
	standard = ".".join([os.sep.join([path, car]), "png"])
	standard_ver = check_version(standard)

	if options.all:
		screenshot = True
	else:
		screenshot = False

	if standard_ver:
		preview = "-".join([os.sep.join([path, car]), "preview.jpg"])
		preview_ver = check_version(preview)

		if (model_ver > standard_ver):
			print "Standard: ACC Model is newer"
		else:
			print "Standard: OK"

		if (preview_ver == None):
			print "Preview : Missing"
			screenshot = True
		elif (preview_ver < 0):
			print "Preview : Not in version control"
			screenshot = True
		elif (preview_ver < standard_ver):
			print "Preview : Out of date"
			screenshot = True
		else:
			print "Preview : OK"

		if options.config and options.run and screenshot:
			screenshot = get_screenshot(module, index, car, None, "1")

			if options.proc:
				# Call alternative script to process images
				os.system(" ".join([options.run, preview]))
			elif _has_PIL:
				process_screenshot(preview)
	else:
		print "Standard: Missing"
	print

	# Check for alternate skins (specific for this car)
	alternates = glob.glob("-".join([os.sep.join([path, car]), "*.png"]))

	if (alternates != None):
		for alternate in alternates:
			alternate_ver=check_version(alternate)

			if options.all:
				screenshot = True
			else:
				screenshot = False

			if (alternate_ver != None):
				(filename,ext) = os.path.splitext(alternate)
				skin = filename.split(car+"-")[1]

				# ignore commonly used textures
				if skin == "int" or skin == "interior" or skin == "speed" or skin == "rpm":
					continue

				if (model_ver > alternate_ver):
					print "Alternate:", os.path.basename(alternate), "(ACC Model is newer)"
				else:
					print "Alternate:", os.path.basename(alternate)

			else:
				continue

			wheelfile = "".join([os.sep.join([path, "wheel3d-"]), skin, ".png"])
			if os.access(wheelfile, os.R_OK):
				print "Custom Wheel: ", os.path.basename(wheelfile)

			if (alternate_ver != None):
				preview = "-".join([filename, "preview.jpg"])
				preview_ver = check_version(preview)

				if (preview_ver == None):
					print "Preview : Missing"
					screenshot = True
				elif (preview_ver < 0):
					print "Preview : Not in version control"
					screenshot = True
				elif (preview_ver < standard_ver):
					print "Preview : Out of date"
					screenshot = True
				else:
					print "Preview : OK"

			if options.config and options.run and screenshot:
				if os.access(wheelfile, os.R_OK):
					screenshot = get_screenshot(module, index, car, skin, "3")
				else:
					screenshot = get_screenshot(module, index, car, skin, "1")

				if options.proc:
					# Call alternative script to process images
					os.system(" ".join([options.run, preview]))
				elif _has_PIL:
					process_screenshot(preview)
			print
	print

