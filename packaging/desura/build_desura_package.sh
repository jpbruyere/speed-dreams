#!/bin/sh

# This script is intended for building SpeedDreams into a Desura package
#
# It should be run from the root of the SpeedDreams repo with a single 
# argument for the location of where to put the temp/clean installation
# ready for buildin the '.mcf'. ie:
#
# $ ./src/tools/scripts/build_desura_package.s /home/simon/for_desura
#
# It should be run twice, once in 64bit and then once in 32bit".

if [ $# = 1 ]; then
	TARGET="$1"
	echo "Using target directory: " $TARGET
else
	echo "Script needs just one argument - path to temp installation"
	exit 0
fi

# clear out current build
if [ -x ./clobber.sh ]; then
	echo "Cleaning out source tree"
	/bin/sh ./clobber.sh
fi	

# do the build and install to tempory directory
cmake -D OPTION_OFFICIAL_ONLY:BOOL=ON -D CMAKE_BUILD_TYPE:STRING=Release -D CMAKE_INSTALL_PREFIX:PATH=$TARGET .
make
make install

# move 64bit binaries and clones PLIB libraries
if [ "$(uname -m)" = "x86_64" ]; then
	echo "64 bit system detected"
	mv $TARGET/games $TARGET/games64
	LIBPATH=lib64

	# Copt ENET into target package
	if [ ! -r $TARGET/$LIBPATH/libenet2 ]; then
		mkdir $TARGET/$LIBPATH/libenet2
	fi
	cp /usr/lib/x86_64-linux-gnu/libenet.so.2 $TARGET/$LIBPATH/libenet2
else
	echo "32 bit system detected"
	LIBPATH=lib

	# Copt ENET into target package
	if [ ! -r $TARGET/$LIBPATH/libenet2 ]; then
		mkdir $TARGET/$LIBPATH/libenet2
	fi
	cp /usr/lib/i386-linux-gnu/libenet.so.2 $TARGET/$LIBPATH/libenet2
fi

# Copy PLIB into target package
if [ ! -r $TARGET/$LIBPATH/libplib1 ]; then
	mkdir $TARGET/$LIBPATH/libplib1
fi
cp /usr/lib/libplibjs.so.1 $TARGET/$LIBPATH/libplib1
cp /usr/lib/libplibsg.so.1 $TARGET/$LIBPATH/libplib1
cp /usr/lib/libplibsl.so.1 $TARGET/$LIBPATH/libplib1
cp /usr/lib/libplibssgaux.so.1 $TARGET/$LIBPATH/libplib1
cp /usr/lib/libplibssg.so.1 $TARGET/$LIBPATH/libplib1
cp /usr/lib/libplibul.so.1 $TARGET/$LIBPATH/libplib1

# Copy informative files
cp $TARGET/share/games/speed-dreams-2/*.txt $TARGET

# Create startup script for running SpeedDreams (from inside Desura)
cat > $TARGET/speed-dreams << EOF
#!/bin/sh
# SpeedDreams unix launch script, automatically selects between 32bit and 64bit 

if [ "\$(uname -m)" = "x86_64" ] && [ -r ./games64 ]; then
	echo "Starting 64bit Speed Dreams."

	GAMEPATH=games64
	LIBPATH=lib64

	if [ -r /usr/lib/x86_64-linux-gnu/libenet.so.2 ]; then
		echo "Using native ENET"
	else
		# Force ENET to be pre-loaded from Desura package
		echo "Using Desura Package's ENET"
		export LD_PRELOAD=\$LD_PRELOAD:./\$LIBPATH/libenet2/libenet.so.2
	fi
else
	echo "Starting 32bit Speed Dreams."

	GAMEPATH=games
	LIBPATH=lib

	if [ -r /usr/lib/i386-linux-gnu/libenet.so.2 ]; then
		echo "Using native ENET"
	else
		# Force ENET to be pre-loaded from Desura package
		echo "Using Desura Package's ENET"
		export LD_PRELOAD=\$LD_PRELOAD:./\$LIBPATH/libenet2/libenet.so.2
	fi
fi

if [ -r /usr/lib/libplibsl.so.1 ]; then
	echo "Using native PLIB"
else
	# Force PLIB to be pre-loaded from Desura package
	echo "Using Desura Package's PLIB"
	export LD_PRELOAD=\$LD_PRELOAD:./\$LIBPATH/libplib1/libplibjs.so.1:./\$LIBPATH/libplib1/libplibsg.so.1:./\$LIBPATH/libplib1/libplibsl.so.1:./\$LIBPATH/libplib1/libplibssgaux.so.1:./\$LIBPATH/libplib1/libplibssg.so.1:./\$LIBPATH/libplib1/libplibul.so.1
fi

./\$GAMEPATH/speed-dreams-2 -ld \$LIBPATH/games/speed-dreams-2/ -dd share/games/speed-dreams-2/ \$@
EOF

# Set as executable
chmod +x $TARGET/speed-dreams

