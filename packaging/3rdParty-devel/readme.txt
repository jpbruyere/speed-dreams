Building the dependencies for Speed-Dreams makes use of CMake's ExternalProject
 module. The source for each is downloaded from each project's site, patched if
 necessary, and built. This can take considerable time and accesses several
 different sites. See the CMakeLists.txt for the exact sites.(search for URL).

 =============================================================================
 Windows
 As of version 2.2, this will download approximately 30MB of source files.
 You will need more than 1GB of free disk space for the build.

 Prerequisites:
 DirectX sDK (June 2010) - needed by SDL and possibly OpenAL
 http://www.microsoft.com/en-us/download/details.aspx?id=6812

  =============================================================================
 OS X
 TODO

   =============================================================================
 Linux
 TODO
 