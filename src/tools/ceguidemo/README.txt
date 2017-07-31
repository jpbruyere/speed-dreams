Document outline :

 I  - Introduction
 II - Installation
    1 - Pre-requisites
      a - Windows
      b - Linux
    2 - Install SD-CEGUI demo
    3 - Run SD-CEGUI demo
      a - Windows
      b - Linux
    4 - Known issues

 A - Build CEGUI 0.7.7 from sources
 B - Install CEED from sources


I - Introduction
-----------------

This is a mock-up / demo application of Speed Dreams,
for experimenting CEGUI features and see if it suits our needs.

It is written in Python, and uses PyCEGUI and PyOpenGL.

CEED is also used to build menu layouts.

II- Installation
----------------

  1 - Pre-requisites
  
    a - Windows

       Download and install the following packages :

       * Python 2.7 (or newer) (but: not tested with Python 3.x)
         from http://www.activestate.com/activepython/downloads

       * PyCEGUI 0.7.5 (or newer)
         from http://pypi.python.org/pypi/PyCEGUI/0.7.5

       * PyOpenGL 3.0.1 (or newer)
         from http://pypi.python.org/pypi/PyOpenGL/3.0.1

       Optional, only needed to develop CEGUI demo app., not to simply run it :
       * last snapshot of CEED (CEGUI Unified Editor, currently under development)
         from http://sourceforge.net/projects/crayzedsgui/files/CEED
		 
	   * MSVC++ 2010 32 bits redistribuable
	     from http://www.microsoft.com/fr-fr/download/details.aspx?id=5555
		 (if you miss it, CEED will refuse to start, and likely issue an error message box
		  saying something like "... prerequisites.pyc ... check() function ...
		  UnicodeDecodeError: 'ascii' codec can't decode byte ???? in position
		  ??: ordinal not in range(128)")

       Note: Actually tested on Windows XP 32 SP2/SP3, 
             with Python 2.7.0, PyCEGUI 0.7.5, PyOpenGL 3.0.1, CEED snapshot 11

    b - Linux

       Install the following packages through the package manager
       of you specific Linux distro. :

       * Python 2.7 (or newer) (but: not tested with Python 3.x)

       * PyCEGUI 0.7.5 (or newer)
    
         Note: If your distro. doesn't ships a pre-built binary package,
               see below (A) for building it from sources.

       * PyOpenGL 3.0.1 (or newer)

       Optional, only needed to develop CEGUI demo app., not to simply run it :
       * CEED (CEGUI Unified Editor, currently under development)

       Note: Actually tested on Linux Mint Debian Edition (AMD 64, Update Pack 5/6), 
             with Python 2.7.3rc1, PyCEGUI 0.7.7 and PyOpenGL 3.0.1

  2 - Install SD-CEGUI demo

     Download the sources from here :

       http://speed-dreams.svn.sourceforge.net/viewvc/speed-dreams/trunk/src/tools/ceguidemo
       (using the "Download GNU tarball at the bottom of the page, 
        and then extracting its contents to where you like)

     or check them out with a subversion client :

       cd ceguidemo
       svn checkout https://speed-dreams.svn.sourceforge.net/svnroot/speed-dreams/trunk/tools/ceguidemo .     
 
  3 - Run SD-CEGUI demo

     a - Windows

       Simply double-click on the shortcut.

     b - Linux

       Run the following command in a terminal :
         python <path/to>/ceguidemo/main.py -l -c

  4 - Known issues

     * Keyboard events seem to be received by menus even when they are hidden or deactivated,
       which gives the following : hit Esc in the Options menu, and you'll quit this menu,
       falling back to the main menu, which will immediately open the "Really quit ?" dialog,
       as if you had hit Esc in the main menu
       (Windows, CEGUI 0.7.5 / Linux, CEGUI 0.7.7)
     * 


A - Build CEGUI 0.7.7 from sources
-------------------------------------------------
 
Warning : Only tested under LMDE UP5, might need some changes to fit your distro.

Download CEGUI-0.7.7.tar.gz 
   from https://sourceforge.net/projects/crayzedsgui/files/CEGUI%20Mk-2/0.7.7/

Install packages :
 * libpython-dev 
 * libboost-dev
 * libboost-python-dev
(and may be also some others, as specified for Ubuntu 12.10 
 at http://www.cegui.org.uk/wiki/index.php/Build_PyCEGUI_from_source_for_Linux :
 * install build-essential pkg-config libtool autoconf
 * libfreetype6-dev libpcre3-dev libpng-dev libmng-dev libjpeg-dev libfreeimage-dev)

cd dev
tar xvfz CEGUI-0.7.7.tar.gz
cd CEGUI-0.7.7

./configure CXXFLAGS="-O3"

make -j2

sudo make install

Add /usr/local/lib to /etc/ld.so.conf.d/x86_64-linux-gnu.conf

sudo ldconfig

B - Install CEED from sources
-------------------------------------------------
 
Warning : Only tested under LMDE UP5, might need some changes to fit your distro.

CEED currently uses the ongoing and unreleased developments of CEGUI 0.8 / 1.0,
so this installation basically consists of 2 steps :
- build current state of (Py)CEGUI 0.8 / 1.0 from sources,
- install CEED

For all of this, follow instructions at 
  http://www.cegui.org.uk/wiki/index.php/CEED#Build_CEGUI_and_PyCEGUI

But you'll probably need to install at least these packages :
 * mercurial (needed for cloning CEGUI and CEED source repositories)
 * libglew-dev (needed for building the PyOpenGLRenderer, not that clear in the CMake
