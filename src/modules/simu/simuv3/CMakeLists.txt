INCLUDE("../../../../cmake/macros.cmake")

SET(_SOURCES SimulationOptions.cpp aero.cpp axle.cpp brake.cpp
             car.cpp categories.cpp collide.cpp differential.cpp
             engine.cpp simu.cpp steer.cpp susp.cpp
             transmission.cpp wheel.cpp
             simuv30.cpp)
SET(_HEADERS aero.h axle.h brake.h carstruct.h differential.h engine.h
             Options.h sim.h steer.h susp.h transmission.h wheel.h
             simuv30.h)
SET(_OTHER_SOURCES ChangeLog)

IF(NOT OPTION_3RDPARTY_SOLID)
	# BEFORE in order to prevent Windows 3rdParty/include/SOLID from taking the flag.
	INCLUDE_DIRECTORIES(BEFORE ../simuv2.1/SOLID-2.0/include)
ELSE(NOT OPTION_3RDPARTY_SOLID)
	ADD_SOLID_INCLUDEDIR()
ENDIF(NOT OPTION_3RDPARTY_SOLID)
ADD_PLIB_INCLUDEDIR()

ADD_INTERFACE_INCLUDEDIR()
ADD_SDLIB_INCLUDEDIR(math tgf robottools portability)

IF(WIN32)
    # DLL export stuff under Windows (to avoid .def file)
    ADD_DEFINITIONS(-DSIMUV30_DLL)
ENDIF(WIN32)

IF(MSVC)
    # Ignore some run-time libs to avoid link time warnings and sometimes even crashes.
    SET(CMAKE_MODULE_LINKER_FLAGS_DEBUG "${CMAKE_MODULE_LINKER_FLAGS_DEBUG} /NODEFAULTLIB:msvcrt.lib")
ENDIF(MSVC)

# Note: Headers and other sources needed for having them available in IDEs.
SD_ADD_LIBRARY(simuv3 MODULE ${_SOURCES} ${_HEADERS} ${_OTHER_SOURCES})

# Might not work with GCC 4.5 or + (non-robot modules crash at 1st reload = after 1 dlclose) 
#SET_TARGET_PROPERTIES(simuv3 PROPERTIES VERSION ${VERSION} SOVERSION 0.0.0)

ADD_SOLID_LIBRARY(simuv3) # Ignored if not OPTION_3RDPARTY_SOLID

ADD_SDLIB_LIBRARY(simuv3 portability tgf robottools solid) # solid ignored if OPTION_3RDPARTY_SOLID

SD_INSTALL_FILES(LIB modules/simu TARGETS simuv3)
