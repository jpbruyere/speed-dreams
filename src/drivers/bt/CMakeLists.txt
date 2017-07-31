INCLUDE(../../../cmake/macros.cmake)

SET(ROBOT_NAME "bt")

SET(ROBOT_SOURCES ${ROBOT_NAME}.cpp cardata.cpp driver.cpp learn.cpp opponent.cpp pit.cpp
	spline.cpp strategy.cpp strategy.h cardata.h driver.h learn.h opponent.h pit.h spline.h)

ROBOT_MODULE(NAME ${ROBOT_NAME} VERSION 1.0.0 SOVERSION 1.0.0
             INTERFACE LEGACY_MIN
             SOURCES ${ROBOT_SOURCES})

# For data associated to the robot module, see data/drivers/bt/CMakeLists.txt
