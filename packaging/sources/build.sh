#!/bin/sh
#
# Script for building source packages
# 
# Usage (example) :
#  cd my/svn/sandbox/tags/2.0.0-rc1
#  ./packaging/sources/build.sh 2.0.0-rc1-r4420
#
# Warning: The generated packages can't be used separately for building :
#          you need to get and extract all of them before building.
#
# copyright  : (C) 2011 onwards Jean-Philippe Meuret
# $Id$

# This program is free software ; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation# either version 2 of the License, or
# (at your option) any later version.

# Check if we are on top of an SD source tree
if [ -f CMakeLists.txt -a -f data/data/credits.xml -a -d cmake -a -d data -a -d src ] ; then

  echo "Building $1 source packages (`date`) ..."
  
  specDir="./packaging/sources"

  echo "* 'Base' package ..."
  tar -c -J -X $specDir/sd-src-exclude.lst --exclude-vcs -T $specDir/sd-src-base.lst -f speed-dreams-src-base-$1.tar.xz

  ls -l speed-dreams-src-base*.tar.xz

  echo "* 'HQ cars and tracks' package ..."
tar -c -J -X $specDir/sd-src-exclude.lst --exclude-vcs -T $specDir/sd-src-hq-cars-and-tracks.lst -f speed-dreams-src-hq-cars-and-tracks-$1.tar.xz

  ls -l speed-dreams-src-hq*.tar.xz

  echo "* 'More HQ cars and tracks' package ..."
tar -c -J -X $specDir/sd-src-exclude.lst --exclude-vcs -T $specDir/sd-src-more-hq-cars-and-tracks.lst -f speed-dreams-src-more-hq-cars-and-tracks-$1.tar.xz

  ls -l speed-dreams-src-more-hq*.tar.xz

  echo "* 'WIP cars and tracks' package ..."
  tar -c -J -X $specDir/sd-src-exclude.lst --exclude-vcs -T $specDir/sd-src-wip-cars-and-tracks.lst -f speed-dreams-src-wip-cars-and-tracks-$1.tar.xz

  ls -l speed-dreams-src-wip*.tar.xz

  echo "* 'unmaintained' package ..."
  tar -c -J -X $specDir/sd-src-exclude.lst --exclude-vcs -T $specDir/sd-src-unmaintained.lst -f speed-dreams-src-unmaintained-$1.tar.xz

  ls -l speed-dreams-src-unmaintained*.tar.xz

  echo "Done (`date`)."
  
else

  echo "Bad current dir for that ; please run from the root folder of an SD source tree."

fi
