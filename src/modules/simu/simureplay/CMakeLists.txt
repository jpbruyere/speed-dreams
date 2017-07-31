IF(NOT OPTION_3RDPARTY_SOLID)

	INCLUDE_DIRECTORIES(BEFORE ../simuv2.1/SOLID-2.0/include)
ELSE(NOT OPTION_3RDPARTY_SOLID)
	ADD_SOLID_INCLUDEDIR()

ENDIF(NOT OPTION_3RDPARTY_SOLID)

INCLUDE(../../../../cmake/macros.cmake)

ADD_SDLIB_INCLUDEDIR(portability)

IF(WIN32)
	# DLL export stuff under Windows (to avoid .def file)
    ADD_DEFINITIONS(-DSIMUREPLAY_DLL)
ENDIF(WIN32)

IF(MSVC)
    # Ignore some run-time libs to avoid link time warnings and sometimes even crashes.
    SET(CMAKE_MODULE_LINKER_FLAGS_DEBUG "${CMAKE_MODULE_LINKER_FLAGS_DEBUG} /NODEFAULTLIB:msvcrt.lib")
ENDIF(MSVC)

SET(SIMU_SOURCES simu.cpp simureplay.cpp)
SET(SIMU_HEADERS sim.h simureplay.h)

IF(NOT OPTION_3RDPARTY_SOLID)
	# BEFORE in order to prevent Windows 3rdParty/include/SOLID from taking the flag.
	INCLUDE_DIRECTORIES(BEFORE SOLID-2.0/include)
ELSE(NOT OPTION_3RDPARTY_SOLID)
	ADD_SOLID_INCLUDEDIR()
ENDIF(NOT OPTION_3RDPARTY_SOLID)
ADD_PLIB_INCLUDEDIR()

ADD_INTERFACE_INCLUDEDIR()
ADD_SDLIB_INCLUDEDIR(math tgf robottools)

# Note: Headers needed for having them available in IDEs.
SD_ADD_LIBRARY(simureplay MODULE ${SIMU_SOURCES} ${SIMU_HEADERS})

# Might not work with GCC 4.5 or + (non-robot modules crash at 1st reload = after 1 dlclose) 
#SET_TARGET_PROPERTIES(simuv2.1 PROPERTIES VERSION ${VERSION} SOVERSION 0.0.0)

ADD_SOLID_LIBRARY(simureplay) # Ignored if not OPTION_3RDPARTY_SOLID

ADD_SDLIB_LIBRARY(simureplay portability tgf robottools solid) # solid ignored if not OPTION_3RDPARTY_SOLID

SD_INSTALL_FILES(LIB modules/simu TARGETS simureplay)
