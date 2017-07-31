Desura Packages Windows (easy)
--
1). Use 'base' binary installer to install to an empty directory. 
2). Delete (or move) the 'unistall' file.
3). VPoint the Desura client to this directory, and make MCF
4). Upload to Desura website and publish
5). Install HQ and More-HQ package
6). Point the Desura client to this directory, and make MCF
7). Upload to Desura website and publish


Desura Packages Linux (complicated)
--
Unfortunately the initial release of SpeedDreams on Linux was done where the
launch command was set to 'speeddreams' in the root of the install, this can 
not be changed and therefore requires a script to work around it.

The 'fix_rpath' patch should first be applied to the source tree, this works
around a problem with the binaries being relocated.

The 'build_desura_package.sh' takes a destination directory as an argument
and will configure, compile and install SpeedDreams into that directory for 
either 32 or 64bit architecture (depending on host).

so from the root of the source tree
--
$ patch -p1 < fix_rpatch.patch
$ build_desura_package.sh ../desura-target
--

The point the Desura client at that directory, make an MCF and upload. This
needs to be done twice, once for 32bit and once for 64bit
